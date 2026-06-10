import { bleService } from './BleService';
import { config } from '../config';

// Types defined in the API Spec
export interface SensorData {
  time: Date;
  amplitude: number;
}

export interface Sensor {
  id: number;
  data: SensorData[];
}

export type Sensors = Sensor[];

export type AudioMode = 0 | 1;  // 0 = accordion, 1 = song + feedback

export const BleEndpoints = {
  LED: 'LED',
  PING: 'PING',
  VOLUME_TOTAL: 'VOLUME_TOTAL',
  SENSOR_THRESHOLD: 'SENSOR_THRESHOLD',
  SENSOR_VOLUME: 'SENSOR_VOLUME',
  MODE: 'MODE',
  SENSITIVITY: 'SENSITIVITY',
  GETLOG: 'GETLOG',
  // Motion mode (default ON in firmware): sound responds to CHANGE in pressure and
  // fades to silence when steady. The SENSITIVITY slider also shifts the motion
  // front/back dead-zone balance, so it keeps working without extra UI.
  MOTION: 'MOTION',           // "1"/"0"
  MOTIONCFG: 'MOTIONCFG',     // "front,back,gain,decay"
  GETCAL: 'GETCAL',
  RANGE: 'RANGE',             // "id,value"
} as const;

// ---------- Diagnostic Log ----------
// Collected from ESP32 via GETLOG command after reconnection.
// Events stored in ring buffer on ESP32 survive BLE disconnects.
export interface DiagnosticEvent {
  index: number;
  timestamp: number;  // millis() on ESP32
  event: string;      // e.g. "BLE_DISC", "HEAP_LOW", "SD_SLOW"
  value: number;
}

type DiagLogCallback = (events: DiagnosticEvent[]) => void;
let diagLogBuffer: DiagnosticEvent[] = [];
let diagLogExpectedCount = 0;
let diagLogCallbacks: DiagLogCallback[] = [];
let diagLogCollecting = false;

// Helper class for simulation state
class SensorSimulator {
  private tick: number = 0;
  private readonly updateSpeed: number = 0.15; // Speed of the walking cycle

  constructor(_count: number) {
    // 
  }

  getNextValues(): number[] {
    this.tick += this.updateSpeed;

    // Simulate Walking Gait
    // Cycle: 0 to 2PI
    
    // Right Foot (0: RF, 2: RB) - Phase 0
    // Left Foot  (1: LF, 3: LB) - Phase PI
    
    // Within a foot: Heel (Back) strikes first, then Toe (Front)
    // Front is slightly delayed relative to Back
    
    const rightPhase = this.tick;
    const leftPhase = this.tick + Math.PI;
    
    // Function to calculate pressure based on phase
    // returns 0-100
    const calcPressure = (phase: number, offset: number) => {
      // Use sine wave, normalize to 0-1, clip negative values (foot in air)
      // Raise to power to make peak narrower (sharper impact)
      const val = Math.sin(phase + offset);
      return val > 0 ? Math.pow(val, 2) * 100 : 0;
    };

    // Offsets
    // Back sensor peaks earlier -> 0 offset
    // Front sensor peaks later -> 0.5 offset (roughly 1/6 of cycle? PI/6 is approx 0.5)
    
    const rb = calcPressure(rightPhase, 0);       // Right Back
    const rf = calcPressure(rightPhase, -0.6);    // Right Front (delayed)
    
    const lb = calcPressure(leftPhase, 0);        // Left Back
    const lf = calcPressure(leftPhase, -0.6);     // Left Front (delayed)

    // Add some random noise
    const noise = () => (Math.random() - 0.5) * 5;

    // Map to id: 0=RF, 1=LF, 2=RB, 3=LB
    return [
      Math.max(0, Math.min(100, rf + noise())),
      Math.max(0, Math.min(100, lf + noise())),
      Math.max(0, Math.min(100, rb + noise())),
      Math.max(0, Math.min(100, lb + noise())),
    ];
  }
}

const simulator = new SensorSimulator(4);
let latestSensorData: Sensors = [];
// Latest calibration snapshot from the firmware (GETCAL response)
export interface CalibrationState {
  base: number[]; thr: number[]; rng: number[];
  locked: number; motion: number; mdf: number; mdb: number;
}
let latestCal: CalibrationState | null = null;
// Raw normalized values (0-100) before calibration subtraction - used for calibration
let latestRawNormalized: number[] = [0, 0, 0, 0];
// Calibration baselines stored in normalized 0-100 range for display adjustment
let calibrationBaselines: number[] = [0, 0, 0, 0];

/**
 * Communication between ESP32 and WebApp
 */
export const EspApi = {
  onDisconnect: (callback: () => void): void => {
    bleService.onDisconnect(callback);
  },
  onReconnect: (callback: (state: 'reconnecting' | 'reconnected' | 'failed') => void): void => {
    bleService.onReconnect(callback);
  },
  connect: async (): Promise<void> => {
    await bleService.connect();
    bleService.subscribeToSensor((jsonString) => {
        try {
            const parsed = JSON.parse(jsonString);
            // FIX: Ignore heartbeat messages from ESP32 (sent when loop() is stalled)
            // These keep BLE alive but contain no sensor data
            if (parsed.hb !== undefined) {
                console.debug('BLE heartbeat received');
                return;
            }
            // Diagnostic log messages from GETLOG command
            if (parsed.log !== undefined) {
                if (parsed.log === 'start') {
                    diagLogBuffer = [];
                    diagLogExpectedCount = parsed.n || 0;
                    diagLogCollecting = true;
                    console.log(`[DiagLog] Receiving ${diagLogExpectedCount} events...`);
                } else if (parsed.log === 'evt' && diagLogCollecting) {
                    diagLogBuffer.push({
                        index: parsed.i,
                        timestamp: parsed.t,
                        event: parsed.e,
                        value: parsed.v,
                    });
                } else if (parsed.log === 'end') {
                    diagLogCollecting = false;
                    console.log(`[DiagLog] Received ${diagLogBuffer.length} events`);
                    console.table(diagLogBuffer);
                    diagLogCallbacks.forEach(cb => cb([...diagLogBuffer]));
                }
                return;
            }
            // Calibration response from GETCAL: {"cal":{...}}
            if (parsed.cal !== undefined) {
                latestCal = parsed.cal;
                return;
            }
            // Compact format: {"t":millis,"s":[raw0..3],"n":[norm0..3]}
            // Prefer the firmware's per-sensor normalized "n" (0-100% of each
            // sensor's OWN learned range) over the old amplitude/4095 approach.
            if (parsed.s && Array.isArray(parsed.s)) {
                const hasN = Array.isArray(parsed.n);
                latestSensorData = parsed.s.map((amplitude: number, index: number) => {
                    const normalized = hasN
                        ? Math.min(100, parsed.n[index])
                        : Math.min(100, (amplitude / 4095) * 100);
                    latestRawNormalized[index] = normalized;
                    // With firmware normalization, baseline subtraction is already done
                    const calibrated = hasN
                        ? normalized
                        : Math.max(0, normalized - (calibrationBaselines[index] || 0));
                    return {
                        id: index,
                        data: [{ time: new Date(), amplitude: calibrated }]
                    };
                });
            } else if (Array.isArray(parsed)) {
                // Legacy format: [{"id":0,"data":[{"time":"...","amplitude":1234}]},...]
                latestSensorData = parsed.map((item: any) => ({
                    id: item.id,
                    data: item.data.map((d: any) => {
                        const normalized = Math.min(100, (Number(d.amplitude) / 4095) * 100);
                        latestRawNormalized[item.id] = normalized;
                        const calibrated = Math.max(0, normalized - (calibrationBaselines[item.id] || 0));
                        return { time: new Date(), amplitude: calibrated };
                    })
                }));
            }
        } catch (e) {
            console.error("Error parsing sensor data", e);
        }
    });
  },
  disconnect: (): void => {
    bleService.disconnect();
  },
  isConnected: (): boolean => {
    return bleService.isConnected();
  },

  subscribeToSensor: (callback: (value: string) => void): void => {
    bleService.subscribeToSensor(callback);
  },
  
  unsubscribeFromSensor: (callback: (value: string) => void): void => {
    bleService.unsubscribeFromSensor(callback);
  },

  write: async (endpoint: string, data: string): Promise<void> => {
    const command = `${endpoint}:${data}`;
    const encoder = new TextEncoder();
    return bleService.write(encoder.encode(command));
  },
  read: async (endpoint: string): Promise<string> => {
     // 1. Send the endpoint name to the device to request data
     const encoder = new TextEncoder();
     // Using "GET:ENDPOINT" convention or just "ENDPOINT" depending on preference. 
     // Given "arduino should get string endpoint", sending just the endpoint might be ambiguous if it looks like a write.
     // But write uses "ENDPOINT:DATA". Read uses "GET:ENDPOINT" seems safer.
     const command = `GET:${endpoint}`;
     await bleService.write(encoder.encode(command));
     
     // 2. Wait for the device to update the characteristic value
     await new Promise(resolve => setTimeout(resolve, 200));

     // 3. Read the response
     const value = await bleService.read();
     const decoder = new TextDecoder('utf-8');
     return decoder.decode(value);
  },

  // First Page
  switchOn: async (isOn: boolean): Promise<void> => {
    // Use string command to avoid null byte (0x00) issue with Arduino String
    const command = `POWER:${isOn ? '1' : '0'}`;
    const encoder = new TextEncoder();
    return bleService.write(encoder.encode(command));
  },
  ping: (): void => {
    // TODO: Implement communication with ESP32
    console.log('ping');
  },
  setVolumeTotal: (volume: number): void => {
    EspApi.write(BleEndpoints.VOLUME_TOTAL, `${volume}`);
  },
  setMode: (mode: AudioMode): void => {
    EspApi.write(BleEndpoints.MODE, `${mode}`);
  },
  // Sensitivity slider: 0=back sensitive, 50=balanced, 100=front sensitive.
  // In motion mode this same slider shifts the front/back motion dead-zone balance
  // (the firmware maps SENSITIVITY to both the exp curve and the motion thresholds).
  setSensitivity: (value: number): void => {
    EspApi.write(BleEndpoints.SENSITIVITY, `${value}`);
  },

  // ---------- Motion mode ----------
  // Motion mode (default ON): sound follows pressure CHANGE, silent when steady.
  setMotionMode: (on: boolean): void => {
    EspApi.write(BleEndpoints.MOTION, on ? '1' : '0');
  },
  // Direct per-zone tuning: front/back dead-zone, gain (x100), decay (x100).
  setMotionConfig: (front: number, back: number, gainx100: number, decayx100: number): void => {
    EspApi.write(BleEndpoints.MOTIONCFG, `${front},${back},${gainx100},${decayx100}`);
  },
  // Manual ceiling override for one sensor's range.
  setRange: (id: number, value: number): void => {
    EspApi.write(BleEndpoints.RANGE, `${id},${value}`);
  },
  // Request the firmware's current calibration/motion state (arrives as {"cal":...}).
  requestCalibration: (): void => {
    EspApi.write(BleEndpoints.GETCAL, '1');
  },
  getLastCalibration: (): CalibrationState | null => latestCal,
  getVolume: (): number => {
    // TODO: Implement communication with ESP32
    console.log('getVolume');
    return 0.0;
  },
  getBatteryHealth: async (): Promise<number> => {
     // TODO: Implement communication with ESP32
     // For now return dummy data
     return 90.0;
  },

  // Second Page
  getSensorsData: (): Sensor[] => {
    // console.log('Checking stubs:', config.useStubs); 
    if (config.useStubs) {
      const values = simulator.getNextValues();
      return values.map((val, index) => ({
        id: index,
        data: [{ time: new Date(), amplitude: val }]
      }));
    }
    return latestSensorData;
  },
  getSensorsThreshold: (): number[] => {
    // TODO: Implement communication with ESP32
    console.log('getSensorsThreshold');
    return [];
  },
  setSensorsThreshold: (thresholds: number[]): void => {
    // Convert from 0-100 range to raw ADC units (0-4095)
    const rawThresholds = thresholds.map(v => Math.round((v / 100) * 4095));
    const command = `SENSOR_THRESHOLD:${rawThresholds.join(',')}`;
    const encoder = new TextEncoder();
    bleService.write(encoder.encode(command));
  },
  getSensorVolume: (id: number): number => {
    // TODO: Implement communication with ESP32
    console.log('getSensorVolume', id);
    return 0.0;
  },
  setSensorVolume: (id: number, volume: number): void => {
    const data = `${id},${volume}`;
    EspApi.write(BleEndpoints.SENSOR_VOLUME, data);
  },

  // ---------- Diagnostic Log ----------
  // Request the ESP32 to send its event log (ring buffer of last 64 events).
  // Events include: BLE disconnects, heap warnings, SD read slowdowns, loop stalls.
  // Returns a Promise that resolves with the log entries.
  requestDiagLog: (): Promise<DiagnosticEvent[]> => {
    return new Promise((resolve) => {
      // Register one-time callback
      const handler: DiagLogCallback = (events) => {
        diagLogCallbacks = diagLogCallbacks.filter(cb => cb !== handler);
        resolve(events);
      };
      diagLogCallbacks.push(handler);

      // Timeout: if no response in 10s, resolve with whatever we have
      setTimeout(() => {
        diagLogCallbacks = diagLogCallbacks.filter(cb => cb !== handler);
        resolve([...diagLogBuffer]);
      }, 10000);

      // Send GETLOG command
      EspApi.write(BleEndpoints.GETLOG, '1');
    });
  },

  // Subscribe to diagnostic log updates (called each time a log is received)
  onDiagLog: (callback: DiagLogCallback): void => {
    diagLogCallbacks.push(callback);
  },

  // Get last received diagnostic log without requesting new one
  getLastDiagLog: (): DiagnosticEvent[] => {
    return [...diagLogBuffer];
  },

  calibrateSensors: async () => {
    // Use the raw normalized values (before calibration subtraction)
    calibrationBaselines = [...latestRawNormalized];
    // Convert from normalized (0-100) back to raw ADC (0-4095)
    const rawBaselines = calibrationBaselines.map(v => Math.round((v / 100) * 4095));
    const command = `CALIBRATE:${rawBaselines.join(',')}`;
    const encoder = new TextEncoder();
    await bleService.write(encoder.encode(command));
  },
};

/**
 * Communication with Vercel Blob
 */
export const blobService = {
  saveSensorData: (sensors: Sensors): void => {
    // TODO: Implement communication with Vercel Blob
    console.log('saveSensorData', sensors);
  },
  getSensorsData: (): Sensors => {
    // TODO: Implement communication with Vercel Blob
    console.log('getSensorsData (Blob)');
    return [];
  },
};
