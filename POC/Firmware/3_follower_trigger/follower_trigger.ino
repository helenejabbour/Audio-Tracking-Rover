// follower_trigger.ino — always-recording ring buffer + level trigger, 2x MAX4466 on ADC1
// Serial commands from Python:
//   ARM,<ms>       arm trigger for <ms>; replies CAP,... A,... B,...  or  TIMEOUT
//   SNAP           capture N samples immediately (noise/clipping check)
//   TH,<abs>,<k>   trigger threshold = abs + k*noise_floor  (ADC counts)
//   STAT           dc / noise floor / threshold / overrun report
#include <Arduino.h>
#include <esp_timer.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #define MIC_A 4      // ADC1_CH3
  #define MIC_B 5      // ADC1_CH4
#else
  #define MIC_A 34     // classic ESP32, ADC1
  #define MIC_B 35
#endif

constexpr int FS   = 10000;               // Hz per channel (if 'over' > 0: try 8000 + python --f1 3200)
constexpr int PRE  = 500, POST = 1000, N = PRE + POST;
constexpr int64_t TS_US = 1000000LL / FS;

enum State { IDLE, ARMED, CAPTURE };
static State st = IDLE;
static uint16_t bufA[N], bufB[N];
static char outBuf[N * 5 + 8];
static int wi = 0, filled = 0, postLeft = 0, trigIdx = 0, overCount = 0;
static uint32_t armUntil = 0, seq = 0;
static int64_t nextT = 0;
static float dcA = 2048, dcB = 2048, env = 0, floorEnv = 5, thAbs = 30, thK = 5;
static char cmdBuf[48]; static int cmdLen = 0;

static void dumpChannel(char tag, const uint16_t* b) {
  char* p = outBuf; *p++ = tag;
  for (int i = 0; i < N; i++) {
    uint16_t v = b[(wi + i) % N];                 // wi = oldest sample -> chronological
    char t[5]; int k = 0;
    if (v == 0) t[k++] = '0';
    while (v) { t[k++] = '0' + v % 10; v /= 10; }
    *p++ = ',';
    while (k) *p++ = t[--k];
  }
  *p++ = '\n';
  Serial.write((const uint8_t*)outBuf, p - outBuf);
}

static void dump() {
  Serial.printf("CAP,%lu,%d,%d,%d,%d,%.1f\n", (unsigned long)seq++, FS, N, trigIdx, overCount, floorEnv);
  dumpChannel('A', bufA); dumpChannel('B', bufB);
  Serial.flush();
}

static void handle(const char* s) {
  if (!strncmp(s, "ARM", 3) && st != CAPTURE) {
    const char* p = strchr(s, ','); int ms = p ? atoi(p + 1) : 3000;
    armUntil = millis() + ms; overCount = 0; env = 0; st = ARMED;
  } else if (!strcmp(s, "SNAP") && st != CAPTURE) {
    overCount = 0; postLeft = N; trigIdx = 0; st = CAPTURE;
  } else if (!strncmp(s, "TH", 2)) {
    sscanf(s, "TH,%f,%f", &thAbs, &thK);
    Serial.printf("TH,%.1f,%.1f\n", thAbs, thK);
  } else if (!strcmp(s, "STAT")) {
    Serial.printf("STAT,dc=%.0f/%.0f,floor=%.1f,thr=%.1f,over=%d\n",
                  dcA, dcB, floorEnv, thAbs + thK * floorEnv, overCount);
  }
}

static void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (cmdLen) { cmdBuf[cmdLen] = 0; handle(cmdBuf); cmdLen = 0; } }
    else if (cmdLen < (int)sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = c;
  }
}

void setup() {
  Serial.begin(921600);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(MIC_A, INPUT); pinMode(MIC_B, INPUT);
  delay(500);
  Serial.printf("READY,%d,%d,%d\n", FS, PRE, POST);
}

void loop() {
  if (nextT == 0) nextT = esp_timer_get_time() + TS_US;
  while (esp_timer_get_time() < nextT) {}                    // fixed-rate slot
  uint16_t a = analogRead(MIC_A), b = analogRead(MIC_B);     // B lags A by a constant -> removed by tau0
  nextT += TS_US;
  int64_t now = esp_timer_get_time();
  if (now > nextT) { overCount++; nextT = now + TS_US; }     // missed slot: flagged, resync

  bufA[wi] = a; bufB[wi] = b;
  if (++wi == N) wi = 0;
  if (filled < N) filled++;

  float xa = a - dcA, xb = b - dcB;
  dcA += (a - dcA) * (1.0f / 512); dcB += (b - dcB) * (1.0f / 512);
  env += (fmaxf(fabsf(xa), fabsf(xb)) - env) * 0.1f;         // ~1 ms envelope
  float thr = thAbs + thK * floorEnv;

  if (st == CAPTURE) {
    if (--postLeft <= 0) { dump(); st = IDLE; filled = 0; env = 0; nextT = 0; }
  } else {
    if (env < thr) floorEnv += (env - floorEnv) * (1.0f / 2048);   // slow noise-floor tracker
    if (st == ARMED) {
      if (filled >= PRE && env > thr) { st = CAPTURE; postLeft = POST; trigIdx = PRE - 1; }
      else if ((int32_t)(millis() - armUntil) > 0) { st = IDLE; Serial.println("TIMEOUT"); }
    }
  }
  if ((wi & 31) == 0) pollSerial();
}