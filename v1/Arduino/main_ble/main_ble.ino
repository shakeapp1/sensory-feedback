/*
  TOM Project
  --- BLE Disconnect Fix ---
  Changes from original:
  1. Larger SD read buffer (256→512 frames) — fewer SPI transactions
  2. SD double-buffering (prefetch) — reads happen while I2S plays previous buffer
  3. Adaptive vTaskDelay (8ms in song mode vs 2ms accordion) — lets loop() run
  4. Zero-allocation BLE command parser (char[] instead of Arduino String)
  5. BLE heartbeat watchdog — bleNotifyTask sends keepalive if loop() is stalled
  6. Heap-low safety — logs warning and throttles when heap drops below 30KB
  */
// ---- WiFi version: BLE replaced by WiFi STA + raw TCP server (port 3232) ----
// The shoe joins a hotspot (PC Mobile-Hotspot / phone, 2.4GHz only).
// Credentials are NOT hardcoded: send over USB-serial:  WIFI:ssid,password
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "driver/i2s_std.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include <Preferences.h>

Preferences prefs;

// ---------- Audio & Pin Configuration ----------
volatile float sensorMaxVol[4] = {1.0f, 1.0f, 1.0f, 1.0f};
volatile int sensorBaselines[4] = {300, 300, 300, 300};
volatile int sensorThresholds[4] = {150, 150, 150, 150};

// ---- MOTION mode: sound responds to CHANGE in pressure, not absolute level ----
// Shira stands and shifts weight most of the time (sensors loaded 48-88% even while
// "walking"), so absolute-level sound is a continuous drone. In motion mode the shoe
// reacts to weight transfer/steps and fades to silence when pressure is steady —
// like real touch adapting. Toggle MOTION:1/0 (saved). Default ON for her.
volatile bool motionMode = true;
// Per-sensor observed min/max (RAW), tracked continuously. Activity is judged ONLY
// by where the current reading sits between THIS sensor's own min and max — the
// gap between them is the sensor's whole vocabulary (Yehuda's principle). Absolute
// values are meaningless across sensors; only the relative position matters.
int obsMin[4] = {4095,4095,4095,4095};
int obsMax[4] = {0,0,0,0};
int prevForce[4] = {0, 0, 0, 0};        // last loop's force per sensor
float motionLevel[4] = {0,0,0,0};       // decaying "movement energy" per sensor
float motionDecay = 0.88f;       // how fast steady pressure fades to silence (lower = faster)
float motionGain  = 2.2f;        // how strongly a change spikes the sound
// Per-sensor motion dead-zone — restores the front/back intensity transfer inside
// motion mode: FRONT of foot is more responsive (lower threshold), HEEL firmer.
// (Yehuda's request — motion mode should still honor front/back.)
int   motionMinDeltaFront = 50;  // front sensors: catch lighter movement
int   motionMinDeltaBack  = 70;  // heel sensors: need firmer movement
int   motionMinDelta[4]   = {70, 50, 50, 70};  // {heel, front, front, heel} per mapping (measured ~25-30 at rest, real motion 140+)

// ---- Power bank keep-alive: sub-audible tone via amp when idle ----
// Starts only after 10s with no sensor activity; stops immediately on press.
// Draws real current through the amplifier so the power bank won't auto-shutoff.
volatile unsigned long lastActivityMs = 0;
volatile bool keepAliveActive = false;
// OFF by default in the WiFi build: the always-on WiFi radio draws steady current
// which usually keeps the power bank awake by itself. Enable only if it still cuts:
// command KEEPALIVE:1 (saved to NVS). User feedback: the hum was grating on these speakers.
volatile bool keepAliveEnabled = false;
const unsigned long KEEPALIVE_IDLE_MS = 4000;   // engage early — this power bank cuts fast
// "Breathing hum": soft audible A3 with slow swell — pleasant by design, draws real
// amp current so the power bank stays awake. (Infrasonic tone at high amp was noisy.)
const float KEEPALIVE_HZ      = 220.0f;  // A3 — warm, unobtrusive
const float KEEPALIVE_LFO_HZ  = 0.25f;   // one breath every ~4 seconds
const float KEEPALIVE_AMP_MIN = 1200.0f;
const float KEEPALIVE_AMP_MAX = 3400.0f;
static float kaPhase = 0.0f;
static float kaLfoPhase = 0.0f;

// ---- Autonomous calibration (3 phases) ----
// Phase 0: boot baseline (strap pressure = zero) — done in setup().
// Phase 1 LEARNING: collect peaks of discrete loading events per sensor
//   (150ms..2s above threshold — stands/turns are filtered out by duration).
// Phase 2 LOCKED: per-sensor range + threshold derived, frozen for the session
//   (predictable mapping for sensory substitution), saved to NVS for next boot.
#define AUTOCAL_EVENTS      10
#define AUTOCAL_TIMEOUT_MS  90000
#define AUTOCAL_MIN_RANGE   400    // ignore learning if peaks tiny (sensors unplugged etc.)
volatile int  sensorRange[4] = {1200, 1200, 1200, 1200};
volatile bool autocalLocked = false;
volatile bool lockChime = false;
static float chimePhase = 0.0f;
static int   chimeSamples = 0;
static int   acPeak[4]  = {0, 0, 0, 0};
static int   acCount[4] = {0, 0, 0, 0};
static int   acBest[4][AUTOCAL_EVENTS];
static unsigned long acStartMs[4] = {0, 0, 0, 0};
static unsigned long acBootMs = 0;
// Continuous range learning AFTER lock: remember record presses, apply only
// during a quiet pause (>=3s below threshold) so the mapping never shifts
// mid-movement. Stretch-only, saved to NVS. (Design agreed with Yehuda.)
static int pendingMax[4] = {0, 0, 0, 0};
static unsigned long lastAboveMs[4] = {0, 0, 0, 0};
volatile float masterVol = 0.5f;  // pleasant autonomous default; adjustable live via BLE
volatile bool systemOn = true;

// Sensitivity curve exponents (controlled via BLE slider 0-100)
// Front uses lower exponent = more sensitive to light touch
// Back uses higher exponent = needs harder press
volatile float frontExp = 0.5f;   // default: sqrt (very responsive)
volatile float backExp  = 2.0f;   // default: squared (needs firm press)

// ---------- Wavetable Synthesis ----------
#define WAVETABLE_SIZE 256
#define NUM_VOICES 4
#define SAMPLE_RATE 22050

// Two timbres for pressure-dependent richness (light press = soft, strong = full accordion)
int16_t wavetableSoft[WAVETABLE_SIZE];
int16_t wavetableRich[WAVETABLE_SIZE];

// PHYSICAL MAPPING (verified 8.6.2026 by press test):
//  idx0 = RIGHT HEEL   idx1 = LEFT FRONT   idx2 = RIGHT FRONT   idx3 = LEFT HEEL
// (Right foot was wired opposite to the original code's assumption.)
const float noteFreqs[NUM_VOICES] = {
  392.00f,  // G4 - idx0 = Right HEEL
  329.63f,  // E4 - idx1 = Left  FRONT
  261.63f,  // C4 - idx2 = Right FRONT
  523.25f   // C5 - idx3 = Left  HEEL
};
// true = front-of-foot sensor (responsive curve); false = heel (firm curve)
const bool sensorIsFront[NUM_VOICES] = { false, true, true, false };

struct Voice {
  float phaseAccumulator;
  float phaseIncrement;
  float phase2;            // second detuned oscillator for natural beating
  float phaseInc2;         // slightly different frequency (± cents)
  volatile float targetVol;
  float currentVol;
  volatile float targetRich;  // pressure-driven timbre richness 0..1
  float currentRich;
  float panL;
  float panR;
};

Voice voices[NUM_VOICES];

// Tremolo LFO for bellows simulation (~5Hz gentle wobble)
float tremoloPhase = 0.0f;
#define TREMOLO_HZ    5.0f
#define TREMOLO_DEPTH 0.08f   // subtle ±8% volume modulation

// ---------- Audio Mode ----------
volatile int audioMode = 0;  // 0 = accordion, 1 = song + enrichment

// Mode 1: Song playback from SD card
File songFile;
bool songFileOpen = false;
#define WAV_HEADER_SIZE 44

// SD file operation flags — BLE callback (Core 0) sets these,
// audio task (Core 1) executes them. Prevents cross-core SPI crash.
volatile bool needOpenSong = false;
volatile bool needCloseSong = false;

// BLE notification runs on Core 0 (same core as BLE stack) to prevent
// cross-core mutex deadlock. loop() writes data here, Core 0 task sends it.
char blePayload[160];   // raw + normalized arrays
volatile bool bleNeedsSend = false;

// FIX #5: BLE heartbeat — track when loop() last updated blePayload
// If loop() is starved for >2s, bleNotifyTask sends a keepalive heartbeat
volatile unsigned long lastBleUpdateMs = 0;
#define BLE_HEARTBEAT_TIMEOUT_MS 2000

// Mode 1: Per-channel frequency-band filtering with hold+decay
// Right foot → right speaker, Left foot → left speaker
// Walking restores filtered frequencies, holds 1.5s, then decays
//
// Low-pass filter at 600Hz: alpha = 2*PI*600 / (2*PI*600 + 22050) ≈ 0.146
#define LP_ALPHA 0.146f
float lpStateL = 0.0f;   // low-pass filter state, left channel
float lpStateR = 0.0f;   // low-pass filter state, right channel

// Base levels (at rest / fully decayed)
#define TREBLE_BASE 0.05f
#define BASS_BASE   0.15f

// Hold + decay timing
#define HOLD_TIME_MS 1500       // hold peak level for 1.5 seconds
#define DECAY_PER_LOOP 0.016f   // decay speed (~1.5s from peak to base at 25ms loop)

// Per-channel band levels (read by audio task)
volatile float trebleLvlR = TREBLE_BASE;  // right speaker treble (sensor 0)
volatile float trebleLvlL = TREBLE_BASE;  // left speaker treble (sensor 1)
volatile float bassLvlR   = BASS_BASE;    // right speaker bass (sensor 2)
volatile float bassLvlL   = BASS_BASE;    // left speaker bass (sensor 3)

// Hold/decay state per sensor band
struct BandHold {
  float peak;                // peak level from last press
  unsigned long lastActive;  // millis() when sensor was last above threshold
};
BandHold holdTrebleR = {TREBLE_BASE, 0};
BandHold holdTrebleL = {TREBLE_BASE, 0};
BandHold holdBassR   = {BASS_BASE, 0};
BandHold holdBassL   = {BASS_BASE, 0};

#define I2S_BCLK_PIN  GPIO_NUM_27
#define I2S_WS_PIN    GPIO_NUM_14
#define I2S_DOUT_PIN  GPIO_NUM_22
#define SD_CS_PIN     5
#define SPI_SCK       18
#define SPI_MISO      19
#define SPI_MOSI      23

// Define sensor pins (ADC pins on ESP32)
const int sensorPins[] = {34, 35, 32, 33};
const int numSensors = 4;
const int ledPin = 2; // Use the appropriate GPIO pin for your setup

i2s_chan_handle_t tx_handle = NULL;

// ---------- BLE State ----------
// ---------- WiFi / TCP state ----------
// Direction reversed: the SHOE connects out to the bridge on the hotspot host.
// (Windows Mobile Hotspot blocks host->client connections, but client->host works.)
// AP MODE: the shoe BROADCASTS its own WiFi. PC/phone connect TO the shoe.
// No external network / hotspot / internet conflict — fully self-contained & stable.
// Shoe net: SSID "ShiraShoe" pass "walkwalk"  shoe IP 192.168.4.1  TCP port 3232.
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
BLEServer* bleServer = NULL;
BLECharacteristic* sensorChar = NULL;
BLECharacteristic* cmdChar = NULL;

// ---- On-device log buffer: never lose data during connection drops ----
// Every sample is pushed to this ring. When connected, wifiTask drains the whole
// backlog (gap-free), then keeps pace live. ~150s buffer at 20Hz; survives any
// stale/reconnect gap (seconds-minutes). Lost only on power-off (RAM).
#define LOG_RING 1500   // ~75s @20Hz; kept modest so BLE/Bluedroid has heap to init
static uint32_t ringT[LOG_RING];
static uint16_t ringS[LOG_RING][4];
static uint8_t  ringN[LOG_RING][4];
volatile int ringWrite = 0;   // next slot to write (loop, core 1)
volatile int ringSent  = 0;   // next slot to send (wifiTask, core 0)
char wifiSsid[33] = "";
char wifiPass[65] = "";
volatile bool needSendCal = false;   // GETCAL: send current calibration over TCP
bool deviceConnected = false;
bool oldDeviceConnected = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define LED_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

// FIX #6: Heap safety threshold
#define HEAP_LOW_THRESHOLD 30000  // 30KB — below this, throttle audio to free CPU

// ---------- Diagnostic Event Log (Ring Buffer) ----------
// Survives BLE disconnects (stored in RAM). Retrieved via "GETLOG" command
// after reconnection to diagnose what happened while disconnected.
// Each entry: 8 bytes. 64 entries = 512 bytes total — negligible RAM cost.
enum EventType : uint8_t {
  EVT_BOOT          = 0,   // System boot (value = restored audio mode)
  EVT_BLE_CONNECT   = 1,   // BLE client connected
  EVT_BLE_DISCONNECT = 2,  // BLE client disconnected
  EVT_HEAP_LOW      = 3,   // Heap below threshold (value = free heap in KB)
  EVT_HEARTBEAT     = 4,   // Heartbeat sent because loop() stalled
  EVT_LOOP_SLOW     = 5,   // loop() took too long (value = gap in ms)
  EVT_SD_READ_SLOW  = 6,   // SD card read took long (value = ms)
  EVT_MODE_CHANGE   = 7,   // Audio mode changed (value = new mode)
  EVT_SD_REWIND     = 8,   // Song file rewound to start
  EVT_SD_FAIL       = 9,   // SD read returned 0 bytes
  EVT_HEAP_SAMPLE   = 10,  // Periodic heap snapshot (value = free heap in KB)
};

struct LogEntry {
  uint32_t timestamp;  // millis()
  uint8_t  event;      // EventType
  uint16_t value;      // context-dependent value
  uint8_t  _pad;       // align to 8 bytes
};

#define LOG_SIZE 64
LogEntry eventLog[LOG_SIZE];
volatile int logHead = 0;   // next write position
volatile int logCount = 0;  // total entries (capped at LOG_SIZE)

void logEvent(EventType evt, uint16_t value = 0) {
  eventLog[logHead].timestamp = millis();
  eventLog[logHead].event = evt;
  eventLog[logHead].value = value;
  logHead = (logHead + 1) % LOG_SIZE;
  if (logCount < LOG_SIZE) logCount++;
}

// Flag: BLE callback sets this, bleNotifyTask sends log entries on Core 0
volatile bool needSendLog = false;

// Track loop() timing for stall detection
volatile unsigned long lastLoopMs = 0;

// ---------- Wavetable Generation ----------
void generateAccordionWavetable() {
  // Pressure-dependent richness: two timbres, crossfaded by press strength.
  // Soft (light press): warm, fundamental-dominant — gentle, never harsh
  // Rich (strong press): full accordion harmonic stack
  const int numHarmonics = 8;
  const float harmonicNum[] = {1, 2, 3, 4, 5, 6, 7, 8};
  const float softAmp[] = {1.0f, 0.30f, 0.12f, 0.04f, 0.0f, 0.0f, 0.0f, 0.0f};
  const float richAmp[] = {1.0f, 0.70f, 0.55f, 0.42f, 0.33f, 0.26f, 0.20f, 0.15f};  // full bright accordion at strong press

  float rawSoft[WAVETABLE_SIZE];
  float rawRich[WAVETABLE_SIZE];
  float peakSoft = 0.0f, peakRich = 0.0f;

  for (int i = 0; i < WAVETABLE_SIZE; i++) {
    float phase = (float)i / WAVETABLE_SIZE * 2.0f * PI;
    float sSoft = 0.0f, sRich = 0.0f;
    for (int h = 0; h < numHarmonics; h++) {
      sSoft += softAmp[h] * sinf(harmonicNum[h] * phase);
      sRich += richAmp[h] * sinf(harmonicNum[h] * phase);
    }
    rawSoft[i] = sSoft;
    rawRich[i] = sRich;
    if (fabsf(sSoft) > peakSoft) peakSoft = fabsf(sSoft);
    if (fabsf(sRich) > peakRich) peakRich = fabsf(sRich);
  }

  // Scale each table to the same peak: crossfade changes timbre, not loudness
  float scaleSoft = (32767.0f * 0.25f) / peakSoft;
  float scaleRich = (32767.0f * 0.25f) / peakRich;
  for (int i = 0; i < WAVETABLE_SIZE; i++) {
    wavetableSoft[i] = (int16_t)(rawSoft[i] * scaleSoft);
    wavetableRich[i] = (int16_t)(rawRich[i] * scaleRich);
  }
  Serial.println("Accordion wavetables generated (soft+rich).");
}

// ---------- Filter State Reset ----------
void resetFilterState() {
  lpStateL = 0.0f;
  lpStateR = 0.0f;
  trebleLvlR = TREBLE_BASE; trebleLvlL = TREBLE_BASE;
  bassLvlR = BASS_BASE;     bassLvlL = BASS_BASE;
  holdTrebleR = {TREBLE_BASE, 0}; holdTrebleL = {TREBLE_BASE, 0};
  holdBassR = {BASS_BASE, 0};    holdBassL = {BASS_BASE, 0};
  Serial.println("Filter initialized (per-channel, 600Hz split, 1.5s hold).");
}

// ---------- Song File Helper ----------
void openSongFile() {
  if (songFileOpen) {
    songFile.close();
    songFileOpen = false;
  }
  songFile = SD.open("/SONG.WAV");
  if (songFile) {
    songFile.seek(WAV_HEADER_SIZE);
    songFileOpen = true;
    Serial.println("SONG.WAV opened");
  } else {
    Serial.println("SONG.WAV not found on SD card!");
  }
}

// ---------- FIX #4: Zero-allocation BLE command parser ----------
// Replaces Arduino String with fixed char buffers to prevent heap fragmentation.
// BLE max write is 512 bytes; 128 is plenty for our commands.
#define CMD_BUF_SIZE 128

// Parse comma-separated ints from a char* buffer. Returns count parsed.
static int parseCsvInts(const char *str, int *out, int maxOut) {
  int count = 0;
  const char *p = str;
  while (*p && count < maxOut) {
    out[count++] = atoi(p);
    // Skip to next comma or end
    while (*p && *p != ',') p++;
    if (*p == ',') p++;
  }
  return count;
}

// ---------- Audio Task (Dual Mode) ----------
// FIX #1: Larger buffer (512 frames) — fewer SD transactions per second
// FIX #2: Double buffering — prefetch next SD block while I2S plays current
void audioTask(void *parameter) {
  const size_t numFrames = 512;   // FIX #1: doubled from 256
  const size_t bufSize = numFrames * 2 * sizeof(int16_t); // stereo 16-bit
  int16_t *buffer = (int16_t *)heap_caps_malloc(bufSize, MALLOC_CAP_DMA);
  // FIX #2: Two song buffers for ping-pong double buffering
  int16_t *songBufA = (int16_t *)heap_caps_malloc(bufSize, MALLOC_CAP_DMA);
  int16_t *songBufB = (int16_t *)heap_caps_malloc(bufSize, MALLOC_CAP_DMA);

  if (buffer == NULL || songBufA == NULL || songBufB == NULL) {
    Serial.println("Failed to allocate audio buffers");
    vTaskDelete(NULL);
    return;
  }

  // Double-buffer state: which buffer has valid prefetched data
  int16_t *songReady = NULL;     // buffer with prefetched data (NULL = none)
  size_t   songReadyBytes = 0;   // how many bytes were prefetched

  i2s_channel_enable(tx_handle);

  // Smoother attack for more natural onset (real bellows take time to build pressure)
  const float attackAlpha  = 0.003f;
  const float releaseAlpha = 0.0008f;

  while (true) {
    // Handle SD file operations on this core (Core 1) to avoid cross-core SPI crash
    if (needCloseSong) {
      needCloseSong = false;
      if (songFileOpen) { songFile.close(); songFileOpen = false; }
      songReady = NULL;  // invalidate prefetch
      Serial.println("Song file closed (audio task)");
    }
    if (needOpenSong) {
      needOpenSong = false;
      openSongFile();
      songReady = NULL;  // invalidate prefetch
      Serial.println("Song file opened (audio task)");
    }

    memset(buffer, 0, bufSize);

    int currentMode = audioMode;

    // ---- Mode 1: Song + frequency filter ----
    if (currentMode == 1 && songFileOpen) {
      // FIX #2: Use prefetched buffer if available, otherwise read now
      int16_t *songBuf;
      size_t bytesRead;
      if (songReady != NULL) {
        // Use the prefetched data — no SD wait!
        songBuf = songReady;
        bytesRead = songReadyBytes;
        songReady = NULL;
      } else {
        // First iteration or after seek — must read synchronously
        songBuf = songBufA;
        unsigned long sdStart = millis();
        bytesRead = songFile.read((uint8_t*)songBuf, bufSize);
        unsigned long sdElapsed = millis() - sdStart;
        if (sdElapsed > 50) {
          logEvent(EVT_SD_READ_SLOW, (uint16_t)sdElapsed);
        }
        if (bytesRead == 0 && songFileOpen) {
          logEvent(EVT_SD_FAIL);
        }
      }

      // FIX #3: Yield after SD read to let loop() run (longer yield in song mode)
      vTaskDelay(pdMS_TO_TICKS(1));  // brief yield between SD read and processing

      if (bytesRead < bufSize) {
        // At song end: pad with silence, rewind for next cycle
        memset((uint8_t*)songBuf + bytesRead, 0, bufSize - bytesRead);
        songFile.seek(WAV_HEADER_SIZE);
        logEvent(EVT_SD_REWIND);
      }

      float mv = masterVol;
      // Per-channel levels: right foot → right speaker, left foot → left speaker
      float tR = trebleLvlR, tL = trebleLvlL;
      float bR = bassLvlR,   bL = bassLvlL;

      for (int f = 0; f < (int)numFrames; f++) {
        int idx = f * 2;
        float rawL = (float)songBuf[idx];
        float rawR = (float)songBuf[idx + 1];

        // Single-pole low-pass filter: splits into bass + treble per channel
        lpStateL += LP_ALPHA * (rawL - lpStateL);
        lpStateR += LP_ALPHA * (rawR - lpStateR);

        // Split each channel into bass and treble bands
        float bassLeft  = lpStateL;
        float bassRight = lpStateR;
        float trebLeft  = rawL - lpStateL;
        float trebRight = rawR - lpStateR;

        // Reconstruct: each speaker controlled by its foot's sensors
        // Left speaker = left foot sensors (1,3)
        // Right speaker = right foot sensors (0,2)
        float outL = (bassLeft * bL + trebLeft * tL) * mv;
        float outR = (bassRight * bR + trebRight * tR) * mv;

        // Clamp output
        int32_t iL = (int32_t)outL;
        int32_t iR = (int32_t)outR;
        buffer[idx]     = (int16_t)(iL > 32767 ? 32767 : (iL < -32768 ? -32768 : iL));
        buffer[idx + 1] = (int16_t)(iR > 32767 ? 32767 : (iR < -32768 ? -32768 : iR));
      }

      // FIX #2: Prefetch next block into the OTHER buffer while I2S plays this one.
      // This way the next iteration won't block on SD read.
      int16_t *prefetchBuf = (songBuf == songBufA) ? songBufB : songBufA;
      songReadyBytes = songFile.read((uint8_t*)prefetchBuf, bufSize);
      if (songReadyBytes < bufSize) {
        memset((uint8_t*)prefetchBuf + songReadyBytes, 0, bufSize - songReadyBytes);
        songFile.seek(WAV_HEADER_SIZE);
        // Re-read from beginning for seamless loop
        size_t remaining = bufSize - songReadyBytes;
        if (remaining > 0 && songFileOpen) {
          songFile.read((uint8_t*)prefetchBuf + songReadyBytes, remaining);
          songReadyBytes = bufSize;
        }
      }
      songReady = prefetchBuf;
    }

    // ---- Mode 0: Accordion wavetable synthesis (dual detuned oscillators + tremolo) ----
    if (currentMode == 0) {
      // Pre-compute tremolo LFO for entire buffer (simple sine, computed once)
      float tPhase = tremoloPhase;
      const float tPhaseInc = (2.0f * PI * TREMOLO_HZ) / (float)SAMPLE_RATE;

      for (int v = 0; v < NUM_VOICES; v++) {
        float phase = voices[v].phaseAccumulator;
        float phaseInc = voices[v].phaseIncrement;
        float phase2 = voices[v].phase2;
        float phaseInc2 = voices[v].phaseInc2;
        float target = voices[v].targetVol;
        float current = voices[v].currentVol;
        float targetR = voices[v].targetRich;
        float currentR = voices[v].currentRich;
        float pL = voices[v].panL;
        float pR = voices[v].panR;
        float localTPhase = tremoloPhase;  // each voice reads same tremolo

        for (int f = 0; f < (int)numFrames; f++) {
          float alpha = (target > current) ? attackAlpha : releaseAlpha;
          current += alpha * (target - current);

          // Smooth the richness so timbre blooms naturally with pressure
          float alphaR = (targetR > currentR) ? attackAlpha : releaseAlpha;
          currentR += alphaR * (targetR - currentR);

          // First oscillator (slightly flat) — soft/rich crossfade by pressure
          int idx0 = (int)phase & (WAVETABLE_SIZE - 1);
          int idx1 = (idx0 + 1) & (WAVETABLE_SIZE - 1);
          float frac = phase - (float)(int)phase;
          float soft1 = (float)wavetableSoft[idx0] + frac * (float)(wavetableSoft[idx1] - wavetableSoft[idx0]);
          float rich1 = (float)wavetableRich[idx0] + frac * (float)(wavetableRich[idx1] - wavetableRich[idx0]);
          float sample1 = soft1 + (rich1 - soft1) * currentR;

          // Second oscillator (slightly sharp) — creates natural beating
          int idx2 = (int)phase2 & (WAVETABLE_SIZE - 1);
          int idx3 = (idx2 + 1) & (WAVETABLE_SIZE - 1);
          float frac2 = phase2 - (float)(int)phase2;
          float soft2 = (float)wavetableSoft[idx2] + frac2 * (float)(wavetableSoft[idx3] - wavetableSoft[idx2]);
          float rich2 = (float)wavetableRich[idx2] + frac2 * (float)(wavetableRich[idx3] - wavetableRich[idx2]);
          float sample2 = soft2 + (rich2 - soft2) * currentR;

          // Mix both oscillators (equal blend for chorus effect)
          float sample = (sample1 + sample2) * 0.5f;

          // Tremolo: clean sine LFO (bellows wobble)
          float tremoloMod = 1.0f + sinf(localTPhase) * TREMOLO_DEPTH;

          float out = sample * current * tremoloMod;

          int bufIdx = f * 2;
          buffer[bufIdx]     += (int16_t)(out * pL);
          buffer[bufIdx + 1] += (int16_t)(out * pR);

          phase += phaseInc;
          if (phase >= (float)WAVETABLE_SIZE) phase -= (float)WAVETABLE_SIZE;
          phase2 += phaseInc2;
          if (phase2 >= (float)WAVETABLE_SIZE) phase2 -= (float)WAVETABLE_SIZE;
          localTPhase += tPhaseInc;
        }

        voices[v].phaseAccumulator = phase;
        voices[v].phase2 = phase2;
        voices[v].currentVol = current;
        voices[v].currentRich = currentR;
      }
      // Update global tremolo phase (advance by numFrames steps)
      tremoloPhase += tPhaseInc * (float)numFrames;
      if (tremoloPhase > 2.0f * PI) tremoloPhase -= 2.0f * PI;
    }

    // ---- Power bank keep-alive: inject sub-audible tone when idle ----
    bool ka = keepAliveEnabled && systemOn && (millis() - lastActivityMs > KEEPALIVE_IDLE_MS);
    if (ka != keepAliveActive) {
      keepAliveActive = ka;
      Serial.println(ka ? "KeepAlive ON (idle >10s)" : "KeepAlive OFF (activity)");
    }
    if (ka) {
      const float kaInc    = 2.0f * PI * KEEPALIVE_HZ / (float)SAMPLE_RATE;
      const float kaLfoInc = 2.0f * PI * KEEPALIVE_LFO_HZ / (float)SAMPLE_RATE;
      for (int f = 0; f < (int)numFrames; f++) {
        // Breathing envelope: gentle swell between AMP_MIN and AMP_MAX
        float breath = 0.5f * (1.0f + sinf(kaLfoPhase));
        float amp = KEEPALIVE_AMP_MIN + breath * (KEEPALIVE_AMP_MAX - KEEPALIVE_AMP_MIN);
        int16_t s = (int16_t)(sinf(kaPhase) * amp);
        int bufIdx = f * 2;
        buffer[bufIdx]     += s;
        buffer[bufIdx + 1] += s;
        kaPhase += kaInc;
        if (kaPhase > 2.0f * PI) kaPhase -= 2.0f * PI;
        kaLfoPhase += kaLfoInc;
        if (kaLfoPhase > 2.0f * PI) kaLfoPhase -= 2.0f * PI;
      }
    }

    // ---- Autocal lock chime: short rising two-note cue (E5 -> A5) ----
    if (lockChime) { lockChime = false; chimeSamples = SAMPLE_RATE / 3; }
    if (chimeSamples > 0) {
      for (int f = 0; f < (int)numFrames && chimeSamples > 0; f++, chimeSamples--) {
        float tt = (float)chimeSamples / ((float)SAMPLE_RATE / 3.0f);  // 1 -> 0
        float freq = (tt > 0.5f) ? 659.25f : 880.0f;
        chimePhase += 2.0f * PI * freq / (float)SAMPLE_RATE;
        if (chimePhase > 2.0f * PI) chimePhase -= 2.0f * PI;
        float env = (tt < 0.12f) ? (tt / 0.12f) : 1.0f;  // fade-out at the very end
        int16_t s = (int16_t)(sinf(chimePhase) * 6000.0f * env);
        int bufIdx = f * 2;
        buffer[bufIdx]     += s;
        buffer[bufIdx + 1] += s;
      }
    }

    // Clamp to prevent overflow (Mode 0 accumulates, Mode 1 already clamped inline)
    for (int i = 0; i < (int)(numFrames * 2); i++) {
      if (buffer[i] > 32767) buffer[i] = 32767;
      if (buffer[i] < -32768) buffer[i] = -32768;
    }

    size_t bytesWritten = 0;
    i2s_channel_write(tx_handle, buffer, bufSize, &bytesWritten, portMAX_DELAY);

    // FIX #3: Adaptive yield — song mode needs more yield for loop() to update BLE
    // Accordion mode: pure CPU math, very fast → short yield
    // Song mode: SD I/O already took time, but loop() still needs its turn
    if (currentMode == 1) {
      vTaskDelay(pdMS_TO_TICKS(8));   // 8ms yield in song mode — lets loop() run reliably
    } else {
      vTaskDelay(pdMS_TO_TICKS(2));   // 2ms in accordion mode — synthesis is lightweight
    }

    // FIX #6: If heap is critically low, add extra delay to reduce pressure
    if (ESP.getFreeHeap() < HEAP_LOW_THRESHOLD) {
      vTaskDelay(pdMS_TO_TICKS(10));  // emergency throttle
    }
  }
}

// ---------- Command processor (shared by TCP and serial config) ----------
// Same command vocabulary as the BLE version: POWER, SENSOR_VOLUME, CALIBRATE,
// SENSOR_THRESHOLD, VOLUME_TOTAL, MODE, GETLOG, SENSITIVITY (+GETCAL added).
void processCommand(const String& value) {
    if (value.length() == 0) return;

    // Check for single-byte legacy LED command (1 byte payload)
    if (value.length() == 1) {
      uint8_t val = (uint8_t)value[0];
      if (val == 1) {
        digitalWrite(ledPin, HIGH);
        systemOn = true;
      } else {
        digitalWrite(ledPin, LOW);
        systemOn = false;
      }
      return;
    }

    // Copy to stack buffer to avoid any heap allocation
    char buf[CMD_BUF_SIZE];
    size_t len = value.length();
    if (len >= CMD_BUF_SIZE) len = CMD_BUF_SIZE - 1;
    memcpy(buf, value.c_str(), len);
    buf[len] = '\0';

    // Find separator ':'
    char *sep = strchr(buf, ':');
    if (sep == NULL) return;

    *sep = '\0';           // split: buf = command, sep+1 = data
    const char *command = buf;
    const char *data = sep + 1;

    if (strcmp(command, "POWER") == 0) {
      int state = atoi(data);
      if (state == 1) {
        digitalWrite(ledPin, HIGH);
        systemOn = true;
        Serial.println("System ON");
      } else {
        digitalWrite(ledPin, LOW);
        systemOn = false;
        Serial.println("System OFF");
      }
    }
    else if (strcmp(command, "SENSOR_VOLUME") == 0) {
      // Data format: "ID,VOLUME"
      const char *comma = strchr(data, ',');
      if (comma != NULL) {
        int id = atoi(data);
        float volume = atof(comma + 1);

        if (id >= 0 && id < 4) {
          if (volume < 0) volume = 0;
          if (volume > 100) volume = 100;

          sensorMaxVol[id] = volume / 100.0f;
          char k[8];
          snprintf(k, sizeof(k), "svol%d", id);
          prefs.putFloat(k, sensorMaxVol[id]);
          Serial.printf("Set Sensor %d Max Vol: %f (saved)\n", id, sensorMaxVol[id]);
        }
      }
    }
    else if (strcmp(command, "CALIBRATE") == 0) {
      int vals[4];
      if (parseCsvInts(data, vals, 4) == 4) {
        for (int i = 0; i < 4; i++) sensorBaselines[i] = vals[i];
        Serial.printf("Calibrated Baselines: %d, %d, %d, %d\n",
          sensorBaselines[0], sensorBaselines[1], sensorBaselines[2], sensorBaselines[3]);
      }
    }
    else if (strcmp(command, "SENSOR_THRESHOLD") == 0) {
      int vals[4];
      if (parseCsvInts(data, vals, 4) == 4) {
        autocalLocked = true;  // manual thresholds override and stop auto-learning
        for (int i = 0; i < 4; i++) {
          sensorThresholds[i] = vals[i];
          char k[8];
          snprintf(k, sizeof(k), "thr%d", i);
          prefs.putInt(k, vals[i]);
        }
        Serial.printf("Thresholds: %d, %d, %d, %d\n",
          sensorThresholds[0], sensorThresholds[1], sensorThresholds[2], sensorThresholds[3]);
      }
    }
    else if (strcmp(command, "VOLUME_TOTAL") == 0) {
      float vol = atof(data);
      if (vol < 0) vol = 0;
      if (vol > 100) vol = 100;
      masterVol = vol / 100.0f;
      prefs.putFloat("mvol", masterVol);
      Serial.printf("Master Volume: %f (saved)\n", masterVol);
    }
    else if (strcmp(command, "MODE") == 0) {
      int mode = atoi(data);
      prefs.putInt("mode", mode);  // persist to NVS — survives resets
      if (mode == 0) {
        audioMode = 0;
        needCloseSong = true;  // Audio task will close file on Core 1
        // Restore accordion frequencies with detuning
        const float dr = powf(2.0f, 4.0f / 1200.0f);
        for (int i = 0; i < NUM_VOICES; i++) {
          float freq = noteFreqs[i];
          voices[i].phaseIncrement = (freq / dr * WAVETABLE_SIZE) / (float)SAMPLE_RATE;
          voices[i].phaseInc2 = (freq * dr * WAVETABLE_SIZE) / (float)SAMPLE_RATE;
          voices[i].targetVol = 0.0f;
        }
        logEvent(EVT_MODE_CHANGE, 0);
        Serial.println("Mode: Accordion (saved)");
      } else if (mode == 1) {
        // Song mode: silence all accordion voices
        for (int i = 0; i < NUM_VOICES; i++) {
          voices[i].targetVol = 0.0f;
        }
        resetFilterState();
        needOpenSong = true;  // Audio task will open file on Core 1
        audioMode = 1;
        logEvent(EVT_MODE_CHANGE, 1);
        Serial.println("Mode: Song (saved, file will open on audio core)");
      }
    }
    else if (strcmp(command, "GETLOG") == 0) {
      // Send diagnostic log via BLE notifications.
      // bleNotifyTask on Core 0 will handle the actual sending.
      needSendLog = true;
      Serial.printf("GETLOG requested (%d entries)\n", logCount);
    }
    else if (strcmp(command, "SENSITIVITY") == 0) {
      // Slider 0-100: 0=back sensitive, 50=balanced, 100=front sensitive
      float s = atof(data);
      if (s < 0) s = 0;
      if (s > 100) s = 100;
      float t = s / 100.0f;
      // Map slider to exponents: higher exponent = less sensitive
      frontExp = 2.0f - t * 1.7f;   // 2.0 at s=0 → 0.3 at s=100
      backExp  = 0.3f + t * 1.7f;   // 0.3 at s=0 → 2.0 at s=100
      prefs.putFloat("fexp", frontExp);
      prefs.putFloat("bexp", backExp);
      Serial.printf("Sensitivity: slider=%d front=%.2f back=%.2f (saved)\n", (int)s, frontExp, backExp);
    }
    else if (strcmp(command, "GETCAL") == 0) {
      needSendCal = true;   // wifiTask sends current calibration as JSON
    }
    else if (strcmp(command, "RANGE") == 0) {
      // Data: "ID,VALUE" — manual ceiling override for one sensor
      const char *comma = strchr(data, ',');
      if (comma != NULL) {
        int id = atoi(data);
        int val = atoi(comma + 1);
        if (id >= 0 && id < 4 && val >= 100) {
          sensorRange[id] = val;
          char k[8];
          snprintf(k, sizeof(k), "rng%d", id);
          prefs.putInt(k, val);
          Serial.printf("Range[%d] set to %d (saved)\n", id, val);
        }
      }
    }
    else if (strcmp(command, "KEEPALIVE") == 0) {
      keepAliveEnabled = (atoi(data) == 1);
      prefs.putBool("kaon", keepAliveEnabled);
      Serial.printf("KeepAlive hum: %s (saved)\n", keepAliveEnabled ? "ON" : "OFF");
    }
    else if (strcmp(command, "MOTION") == 0) {
      motionMode = (atoi(data) == 1);
      prefs.putBool("motion", motionMode);
      for (int i = 0; i < 4; i++) { motionLevel[i] = 0; prevForce[i] = 0; }
      Serial.printf("Motion mode: %s (saved)\n", motionMode ? "ON" : "OFF");
    }
    else if (strcmp(command, "MOTIONCFG") == 0) {
      // "front,back,gainx100,decayx100" — separate front/back dead-zones (Yehuda).
      // Falls back to "minDelta,gainx100,decayx100" (uniform) for compatibility.
      int v[4];
      if (parseCsvInts(data, v, 4) == 4) {
        motionMinDeltaFront = v[0];
        motionMinDeltaBack  = v[1];
        motionGain = v[2] / 100.0f;
        motionDecay = v[3] / 100.0f;
        for (int i = 0; i < 4; i++) motionMinDelta[i] = sensorIsFront[i] ? motionMinDeltaFront : motionMinDeltaBack;
        Serial.printf("MotionCfg: front=%d back=%d gain=%.2f decay=%.2f\n",
          motionMinDeltaFront, motionMinDeltaBack, motionGain, motionDecay);
      } else {
        int v3[3];
        if (parseCsvInts(data, v3, 3) == 3) {
          for (int i = 0; i < 4; i++) motionMinDelta[i] = v3[0];
          motionGain = v3[1] / 100.0f;
          motionDecay = v3[2] / 100.0f;
          Serial.printf("MotionCfg(uniform): minDelta=%d gain=%.2f decay=%.2f\n", v3[0], motionGain, motionDecay);
        }
      }
    }
}

// ---------- WiFi/TCP server task (Core 0) ----------
// Replaces bleNotifyTask: maintains WiFi STA connection, accepts one TCP client,
// streams sensor payloads, receives command lines, serves GETLOG/GETCAL.
// Event type names for log output
static const char* evtNames[] = {
  "BOOT", "BLE_CONN", "BLE_DISC", "HEAP_LOW", "HEARTBEAT",
  "LOOP_SLOW", "SD_SLOW", "MODE_CHG", "SD_REWIND", "SD_FAIL", "HEAP_SNAP"
};

// ---- BLE connection callbacks ----
class ShoeServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) {
    deviceConnected = true; logEvent(EVT_BLE_CONNECT); Serial.println("BLE: connected");
  }
  void onDisconnect(BLEServer* s) {
    deviceConnected = false; logEvent(EVT_BLE_DISCONNECT); Serial.println("BLE: disconnected");
    s->getAdvertising()->start();
  }
};
class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) {
    String v = c->getValue();
    if (v.length() > 0) processCommand(v);
  }
};
static inline bool bleSend(const char* s) {
  if (!deviceConnected || sensorChar == NULL) return false;
  sensorChar->setValue((uint8_t*)s, strlen(s));
  sensorChar->notify();
  return true;
}
void bleTask(void *parameter) {
  while (true) {
    if (!deviceConnected) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
    if (needSendCal) {
      needSendCal = false;
      char calMsg[200];
      snprintf(calMsg, sizeof(calMsg),
        "{\"cal\":{\"base\":[%d,%d,%d,%d],\"thr\":[%d,%d,%d,%d],\"rng\":[%d,%d,%d,%d],\"locked\":%d}}",
        (int)sensorBaselines[0], (int)sensorBaselines[1], (int)sensorBaselines[2], (int)sensorBaselines[3],
        (int)sensorThresholds[0], (int)sensorThresholds[1], (int)sensorThresholds[2], (int)sensorThresholds[3],
        (int)sensorRange[0], (int)sensorRange[1], (int)sensorRange[2], (int)sensorRange[3],
        autocalLocked ? 1 : 0);
      bleSend(calMsg); vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (needSendLog) {
      needSendLog = false;
      int count = logCount;
      int start = (count < LOG_SIZE) ? 0 : logHead;
      char hdr[60]; snprintf(hdr, sizeof(hdr), "{\"log\":\"start\",\"n\":%d}", count);
      bleSend(hdr); vTaskDelay(pdMS_TO_TICKS(25));
      for (int i = 0; i < count && deviceConnected; i++) {
        int idx = (start + i) % LOG_SIZE;
        LogEntry &e = eventLog[idx];
        const char *name = (e.event < sizeof(evtNames)/sizeof(evtNames[0])) ? evtNames[e.event] : "?";
        char line[80];
        snprintf(line, sizeof(line), "{\"log\":\"evt\",\"i\":%d,\"t\":%lu,\"e\":\"%s\",\"v\":%u}", i, e.timestamp, name, e.value);
        bleSend(line); vTaskDelay(pdMS_TO_TICKS(25));
      }
      bleSend("{\"log\":\"end\"}");
      vTaskDelay(pdMS_TO_TICKS(40));
      continue;
    }
    int sent = 0;
    while (ringSent != ringWrite && sent < 60 && deviceConnected) {
      int r = ringSent;
      char line[160];
      snprintf(line, sizeof(line), "{\"t\":%lu,\"s\":[%u,%u,%u,%u],\"n\":[%u,%u,%u,%u]}",
        (unsigned long)ringT[r],
        ringS[r][0], ringS[r][1], ringS[r][2], ringS[r][3],
        ringN[r][0], ringN[r][1], ringN[r][2], ringN[r][3]);
      if (!bleSend(line)) break;
      ringSent = (r + 1) % LOG_RING;
      sent++;
      vTaskDelay(pdMS_TO_TICKS(8));
    }
    vTaskDelay(pdMS_TO_TICKS(sent > 0 ? 2 : 20));
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

  // Initialize sensor pins
  analogReadResolution(12); // Ensure we use 12-bit resolution matching new sketch
  for(int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Boot auto-calibration: whatever pressure exists at power-on (e.g. brace strap)
  // becomes the zero point. Only pressure ABOVE it (real steps) triggers sound.
  delay(300);  // let ADC settle
  for (int i = 0; i < numSensors; i++) {
    long sum = 0;
    for (int k = 0; k < 8; k++) { sum += analogRead(sensorPins[i]); delay(2); }
    sensorBaselines[i] = (int)(sum / 8);
  }
  Serial.printf("Boot baselines: %d, %d, %d, %d\n",
    sensorBaselines[0], sensorBaselines[1], sensorBaselines[2], sensorBaselines[3]);

  // ---------- SD Card Setup (kept for future use) ----------
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Init Failed!");
  } else {
    Serial.println("SD Card Ready");
  }

  // ---------- Restore saved mode from NVS ----------
  prefs.begin("audio", false);
  audioMode = prefs.getInt("mode", 0);  // default: accordion

  // Restore saved tuning from NVS (survives power cycles; baselines stay fresh per boot)
  // Only "taste" preferences persist (volume, sensitivity curve, mode). NOT thresholds
  // or ranges — those depend on the sensors' position in the brace TODAY and MUST be
  // learned fresh every power-on (baseline above + autocal from first ~10 steps).
  masterVol = prefs.getFloat("mvol", masterVol);
  frontExp  = prefs.getFloat("fexp", frontExp);
  backExp   = prefs.getFloat("bexp", backExp);
  for (int i = 0; i < 4; i++) {
    char k[8];
    snprintf(k, sizeof(k), "svol%d", i);
    sensorMaxVol[i] = prefs.getFloat(k, sensorMaxVol[i]);
  }
  // thresholds + ranges start at safe defaults and get learned this session
  autocalLocked = false;   // re-learn ranges/thresholds every boot
  keepAliveEnabled = prefs.getBool("kaon", false);
  motionMode = prefs.getBool("motion", true);   // default ON — fits Shira's standing pattern
  Serial.printf("Fresh-calibration boot. mvol %.2f motion %d (thresholds/ranges learn this session)\n",
    masterVol, motionMode ? 1 : 0);
  Serial.printf("Restored mode: %d (%s)\n", audioMode, audioMode == 1 ? "Song" : "Accordion");
  if (audioMode == 1) {
    needOpenSong = true;  // Audio task will open song file after starting
  }

  // ---------- Synthesis Setup ----------
  generateAccordionWavetable();
  resetFilterState();
  // Detuning: ±4 cents creates the classic accordion "beating" between two reeds
  // cents-to-ratio: 2^(cents/1200)
  const float detuneRatio = powf(2.0f, 4.0f / 1200.0f);  // ~1.00231

  for (int i = 0; i < NUM_VOICES; i++) {
    float freq = noteFreqs[i];
    voices[i].phaseAccumulator = 0.0f;
    voices[i].phaseIncrement = (freq / detuneRatio * WAVETABLE_SIZE) / (float)SAMPLE_RATE;  // slightly flat
    voices[i].phase2 = 0.0f;
    voices[i].phaseInc2 = (freq * detuneRatio * WAVETABLE_SIZE) / (float)SAMPLE_RATE;       // slightly sharp
    voices[i].targetVol = 0.0f;
    voices[i].currentVol = 0.0f;
    // Right foot sensors (0,2) -> right speaker, Left foot sensors (1,3) -> left speaker
    voices[i].panL = (i == 1 || i == 3) ? 1.0f : 0.0f;
    voices[i].panR = (i == 0 || i == 2) ? 1.0f : 0.0f;
  }

  // ---------- I2S Setup ----------
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, &tx_handle, NULL);
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(22050), // Matches typical WAV sample rate
      .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
      .gpio_cfg = {
        .mclk = I2S_GPIO_UNUSED,
        .bclk = I2S_BCLK_PIN,
        .ws = I2S_WS_PIN,
        .dout = I2S_DOUT_PIN,
        .din = I2S_GPIO_UNUSED
      }
  };
  i2s_channel_init_std_mode(tx_handle, &std_cfg);

  // Audio on Core 1 (same core as Arduino loop) — frees Core 0 for BLE stack
  // Priority 3 > loop's 1, so audio gets CPU when needed but yields on I2S DMA block
  // FIX: Increased stack to 20480 for larger buffers
  xTaskCreatePinnedToCore(audioTask, "AudioTask", 20480, NULL, 3, NULL, 1);
  Serial.println("Audio Task Started.");

  // ---------- WiFi Init (replaces BLE) ----------
  // Credentials from NVS; set/replace over USB-serial with:  WIFI:ssid,password
  // ---------- BLE init ----------
  BLEDevice::init("ShiraShoe");
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ShoeServerCallbacks());
  BLEService* svc = bleServer->createService(SERVICE_UUID);
  sensorChar = svc->createCharacteristic(SENSOR_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  sensorChar->addDescriptor(new BLE2902());
  cmdChar = svc->createCharacteristic(LED_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  cmdChar->setCallbacks(new CmdCallbacks());
  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("BLE started: ShiraShoe (advertising).");
  xTaskCreatePinnedToCore(bleTask, "BLETask", 6144, NULL, 2, NULL, 0);

  // Log boot event with restored mode
  logEvent(EVT_BOOT, (uint16_t)audioMode);
  lastLoopMs = millis();
}

void loop() {
  // ---- Read Sensors ----
  int sensorValues[numSensors];
  unsigned long timestamp = millis();

  for (int i = 0; i < numSensors; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    // Track this sensor's own observed range (min..max). Slow max-relax so a one-off
    // spike doesn't permanently inflate the range; min follows true rest when foot lifts.
    if (sensorValues[i] < obsMin[i]) obsMin[i] = sensorValues[i];
    if (sensorValues[i] > obsMax[i]) obsMax[i] = sensorValues[i];
  }

  // TEMP DEBUG (sensor diagnosis): print raw values every 250ms — remove after testing
  static unsigned long lastDbgPrint = 0;
  if (timestamp - lastDbgPrint >= 250) {
    lastDbgPrint = timestamp;
    Serial.printf("FSR: %4d %4d %4d %4d | base: %d %d %d %d | thr: %d %d %d %d\n",
      sensorValues[0], sensorValues[1], sensorValues[2], sensorValues[3],
      (int)sensorBaselines[0], (int)sensorBaselines[1], (int)sensorBaselines[2], (int)sensorBaselines[3],
      (int)sensorThresholds[0], (int)sensorThresholds[1], (int)sensorThresholds[2], (int)sensorThresholds[3]);
  }

  // Keep-alive activity tracking: any pressed sensor counts as activity
  for (int i = 0; i < numSensors; i++) {
    if (sensorValues[i] - sensorBaselines[i] > sensorThresholds[i]) {
      lastActivityMs = timestamp;
      break;
    }
  }

  // ---- Autonomous calibration: phase 1 learning → phase 2 lock ----
  if (!autocalLocked) {
    if (acBootMs == 0) acBootMs = timestamp;
    for (int i = 0; i < numSensors; i++) {
      int force = sensorValues[i] - sensorBaselines[i];
      if (force < 0) force = 0;
      if (acStartMs[i] == 0) {
        if (force > sensorThresholds[i]) { acStartMs[i] = timestamp; acPeak[i] = force; }
      } else {
        if (force > acPeak[i]) acPeak[i] = force;
        if (force < sensorThresholds[i] / 2 + 1) {
          unsigned long durMs = timestamp - acStartMs[i];
          // Discrete step events only: stands/turns (>2s) and blips (<150ms) dropped
          if (durMs >= 150 && durMs <= 2000 && acCount[i] < AUTOCAL_EVENTS) {
            acBest[i][acCount[i]++] = acPeak[i];
          }
          acStartMs[i] = 0;
        }
      }
    }
    bool enough = true;
    for (int i = 0; i < numSensors; i++) if (acCount[i] < AUTOCAL_EVENTS) enough = false;
    if (enough || (timestamp - acBootMs > AUTOCAL_TIMEOUT_MS)) {
      int adapted = 0;
      for (int i = 0; i < numSensors; i++) {
        if (acCount[i] >= 3) {
          for (int a = 0; a < acCount[i]; a++)
            for (int b = a + 1; b < acCount[i]; b++)
              if (acBest[i][b] < acBest[i][a]) { int tmp = acBest[i][a]; acBest[i][a] = acBest[i][b]; acBest[i][b] = tmp; }
          int rng = acBest[i][acCount[i] - 1];  // her TRUE max — full spectrum is hers (Yehuda)
          if (rng >= AUTOCAL_MIN_RANGE) {
            sensorRange[i] = rng;
            int th = rng * 12 / 100;
            if (th < 60) th = 60;
            sensorThresholds[i] = th;
            char k[8];
            snprintf(k, sizeof(k), "rng%d", i);
            prefs.putInt(k, rng);
            snprintf(k, sizeof(k), "thr%d", i);
            prefs.putInt(k, th);
            adapted++;
          }
        }
      }
      autocalLocked = true;
      lockChime = true;  // audible cue: calibration locked
      Serial.printf("AUTOCAL LOCKED (%d adapted): rng %d,%d,%d,%d thr %d,%d,%d,%d\n",
        adapted, (int)sensorRange[0], (int)sensorRange[1], (int)sensorRange[2], (int)sensorRange[3],
        (int)sensorThresholds[0], (int)sensorThresholds[1], (int)sensorThresholds[2], (int)sensorThresholds[3]);
    }
  } else {
    // ---- Post-lock continuous range learning (stretch-only, applied in pauses) ----
    // A record press is remembered but applied only after >=3s of quiet on that
    // sensor, so the mapping never shifts mid-movement. (Agreed with Yehuda.)
    for (int i = 0; i < numSensors; i++) {
      int force = sensorValues[i] - sensorBaselines[i];
      if (force < 0) force = 0;
      if (force > sensorThresholds[i]) {
        lastAboveMs[i] = timestamp;
        if (force > sensorRange[i] && force > pendingMax[i]) pendingMax[i] = force;
      } else if (pendingMax[i] > 0 && timestamp - lastAboveMs[i] > 3000) {
        sensorRange[i] = pendingMax[i];   // new personal record becomes the new 100%
        char k[8];
        snprintf(k, sizeof(k), "rng%d", i);
        prefs.putInt(k, sensorRange[i]);
        Serial.printf("Range stretched: sensor %d -> %d (applied in pause)\n", i, sensorRange[i]);
        pendingMax[i] = 0;
      }
    }
  }

  // ---- Audio Logic (mode-dependent) ----
  if (systemOn) {
    int currentMode = audioMode;

    if (currentMode == 0) {
      // Mode 0: Accordion - all 4 voices play C Major
      for (int i = 0; i < numSensors; i++) {
        int force = sensorValues[i] - sensorBaselines[i];
        if (force < 0) force = 0;

        // MOTION mode: drive sound from pressure CHANGE, decaying to silence when steady
        if (motionMode) {
          int delta = force - prevForce[i];
          if (delta < 0) delta = -delta;            // rise or fall both count as motion
          if (delta < motionMinDelta[i]) delta = 0; // per-sensor dead-zone (front lower, heel higher)
          float spike = (float)delta / (float)sensorRange[i] * motionGain;
          motionLevel[i] = motionLevel[i] * motionDecay + spike;
          if (motionLevel[i] > 1.0f) motionLevel[i] = 1.0f;
          prevForce[i] = force;

          if (motionLevel[i] > 0.03f && force > sensorThresholds[i] / 3) {
            // Front/back intensity transfer kept ALIVE in motion mode:
            // front (low exp) blooms fast, heel (high exp) needs stronger motion.
            float curve = sensorIsFront[i] ? frontExp : backExp;
            float nf = powf(motionLevel[i], curve);
            voices[i].targetVol = (0.3f + nf * 0.7f) * sensorMaxVol[i] * masterVol;
            if (voices[i].targetVol > 1.0f) voices[i].targetVol = 1.0f;
            voices[i].targetRich = nf;
          } else {
            voices[i].targetVol = 0.0f;
            voices[i].targetRich = 0.0f;
          }
          continue;
        }

        if (force > sensorThresholds[i]) {
          // Map force against this sensor's LEARNED range (placement-independent feel)
          float normalizedForce = (float)force / (float)sensorRange[i];
          if (normalizedForce > 1.0f) normalizedForce = 1.0f;

          // Sensitivity curve by ACTUAL position (front responsive, heel firm)
          float exp = sensorIsFront[i] ? frontExp : backExp;
          normalizedForce = powf(normalizedForce, exp);

          float baseVolume = 0.3f + (normalizedForce * 0.7f);
          voices[i].targetVol = baseVolume * sensorMaxVol[i] * masterVol;
          if (voices[i].targetVol > 1.0f) voices[i].targetVol = 1.0f;
          voices[i].targetRich = normalizedForce;  // stronger press = richer timbre
        } else {
          voices[i].targetVol = 0.0f;
          voices[i].targetRich = 0.0f;
        }
      }
    } else {
      // Mode 1: Per-channel frequency filter with hold+decay
      // Each sensor controls one band on one speaker
      unsigned long now = millis();

      // Helper: update a band level with hold+decay logic
      // Returns the new output level
      // sensExp: sensitivity exponent (front=low → responsive, back=high → firm)
      #define UPDATE_BAND(sensorIdx, baseVal, hold, outVar, sensExp) do { \
        int force = sensorValues[sensorIdx] - sensorBaselines[sensorIdx]; \
        if (force < 0) force = 0; \
        if (force > sensorThresholds[sensorIdx]) { \
          float nf = (float)force / (float)sensorRange[sensorIdx]; \
          if (nf > 1.0f) nf = 1.0f; \
          nf = powf(nf, sensExp); \
          float level = baseVal + nf * (1.0f - baseVal); \
          hold.peak = level; \
          hold.lastActive = now; \
          outVar = level; \
        } else { \
          unsigned long elapsed = now - hold.lastActive; \
          if (elapsed < HOLD_TIME_MS) { \
            outVar = hold.peak; \
          } else { \
            float decayed = hold.peak - DECAY_PER_LOOP; \
            if (decayed < baseVal) decayed = baseVal; \
            hold.peak = decayed; \
            outVar = decayed; \
          } \
        } \
      } while(0)

      float fExp = frontExp;  // read volatile once
      float bExp = backExp;
      // FRONT sensors -> treble (corrected mapping: R-front=idx2, L-front=idx1)
      UPDATE_BAND(2, TREBLE_BASE, holdTrebleR, trebleLvlR, fExp);
      UPDATE_BAND(1, TREBLE_BASE, holdTrebleL, trebleLvlL, fExp);
      // HEEL sensors -> bass (corrected mapping: R-heel=idx0, L-heel=idx3)
      UPDATE_BAND(0, BASS_BASE, holdBassR, bassLvlR, bExp);
      UPDATE_BAND(3, BASS_BASE, holdBassL, bassLvlL, bExp);
    }
  } else {
    for (int i = 0; i < NUM_VOICES; i++) {
      voices[i].targetVol = 0.0f;
    }
    trebleLvlR = TREBLE_BASE; trebleLvlL = TREBLE_BASE;
    bassLvlR = BASS_BASE;     bassLvlL = BASS_BASE;
  }

  // ---- Loop stall detection ----
  // If loop() hasn't run in >200ms, something starved it (audio task or SD)
  unsigned long loopGap = timestamp - lastLoopMs;
  if (lastLoopMs > 0 && loopGap > 200) {
    logEvent(EVT_LOOP_SLOW, (uint16_t)min(loopGap, (unsigned long)65535));
  }
  lastLoopMs = timestamp;

  // ---- BLE Logic ----
  // Write sensor data to shared buffer; Core 0 bleNotifyTask does the actual BLE send.
  // This prevents loop() from holding BLE mutex while audio task preempts it.
  if (deviceConnected && !bleNeedsSend) {
    // Normalized per-sensor values: 0-100% of each sensor's PERSONAL range
    // (placement-independent — Yehuda's normalization principle)
    int nrm[4];
    for (int i = 0; i < numSensors; i++) {
      int force = sensorValues[i] - sensorBaselines[i];
      if (force < 0) force = 0;
      long p = (long)force * 100 / (sensorRange[i] > 0 ? sensorRange[i] : 1);
      nrm[i] = (p > 100) ? 100 : (int)p;
    }
    // Push to on-device ring buffer (drained by wifiTask; no data lost on drops)
    int w = ringWrite;
    ringT[w] = timestamp;
    for (int i = 0; i < 4; i++) { ringS[w][i] = (uint16_t)sensorValues[i]; ringN[w][i] = (uint8_t)nrm[i]; }
    int nw = (w + 1) % LOG_RING;
    ringWrite = nw;
    if (nw == ringSent) ringSent = (ringSent + 1) % LOG_RING;  // overflow: drop oldest
    lastBleUpdateMs = millis();
  }

  // ---- Serial command channel (USB): WIFI:ssid,password / IP? / any command ----
  static String serialLine = "";
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLine.length() > 0) {
        if (serialLine.startsWith("IP?")) {
          Serial.printf("BLE ShiraShoe — connected: %d\n", deviceConnected ? 1 : 0);
        } else {
          processCommand(serialLine);  // full command set works over USB too
        }
        serialLine = "";
      }
    } else if (serialLine.length() < 200) {
      serialLine += c;
    }
  }

  // Heap monitoring (every ~5 seconds)
  // FIX #6: More detailed logging + warning threshold
  static unsigned long lastHeapLog = 0;
  if (millis() - lastHeapLog > 5000) {
    lastHeapLog = millis();
    uint32_t freeHeap = ESP.getFreeHeap();
    uint32_t minHeap = ESP.getMinFreeHeap();
    Serial.printf("Free heap: %d  Min: %d  Mode: %d\n", freeHeap, minHeap, audioMode);
    // Log heap snapshot every 5s (value = KB free)
    logEvent(EVT_HEAP_SAMPLE, (uint16_t)(freeHeap / 1024));
    if (freeHeap < HEAP_LOW_THRESHOLD) {
      logEvent(EVT_HEAP_LOW, (uint16_t)(freeHeap / 1024));
      Serial.println("WARNING: Heap critically low! BLE may disconnect.");
    }
  }

  delay(50);
}
