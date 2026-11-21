/*
  esp32_acoustic_frontend_bt.ino
  Same measurement chain as esp32_acoustic_frontend.ino (paired Goertzel amplitudes A- / A+ at phi_c -/+ delta),
  plus wireless: every line goes to USB serial AND, while a client is connected, to Bluetooth Classic (SPP).
  Commands are accepted from either link. The protocol is unchanged, so esp_logger.py works as-is
  (--port = the Bluetooth COM port / /dev/rfcomm0).

  Amplitude (peak, ADC counts):  A = (2/N) | sum (x[n]-mean) e^{-j 2 pi f n / fs} |     (report Eq. goertzel)
  f0 and fref are bin-centred (f*N/fs integer); fref is an off-tone bin used as noise-floor / SNR reference.

  Lines (ASCII)
    ESP32 -> host
      C,<fs>,<N>,<f0>,<fref>,<settle_ms>,<adc_us>        config (boot and on '?')
      M,<seq>,<t_ms>,<dt_pair_ms>,<phi_c>,<dlt>,<A_m>,<A_p>,<Aref_m>,<Aref_p>,<pp_m>,<pp_p>,<flags>
         flags: bit0 valid, bit1 clipped, bit2 timing overrun (>5% of samples started late)
    host -> ESP32:  P<deg> arm centre | D<deg> dither | S<ms> servo settle | ?  print config
  Bluetooth packets are lost while no client is connected; seq gaps in the CSV show it (logger 'dropped').
*/
#include <Arduino.h>
#include <stdarg.h>
#include <ESP32Servo.h>

#ifndef USE_BT
#define USE_BT 1                 // 0 = USB only (for A/B noise-floor tests)
#endif
#if USE_BT
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Classic Bluetooth is not enabled: use a classic ESP32 (not S2/S3/C3) and board 'ESP32 Dev Module'"
#endif
BluetoothSerial SerialBT;
constexpr const char* BT_NAME = "AcousticFollower";   // name shown when pairing
#endif

// ---------------- pins ----------------
constexpr int PIN_MIC   = 34;    // MAX4466 OUT -> ADC1 pin
constexpr int PIN_SERVO = 32;    // SG90 signal

// ---------------- acquisition ----------------
constexpr int FS_HZ   = 16000;
constexpr int N_SAMP  = 512;     // 32 ms window
constexpr int F0_HZ   = 4000;    // beacon tone
constexpr int FREF_HZ = 3000;    // noise reference, not a harmonic of f0
constexpr int K0   = (int)((long)F0_HZ   * N_SAMP / FS_HZ);
constexpr int KREF = (int)((long)FREF_HZ * N_SAMP / FS_HZ);
static_assert((long)F0_HZ   * N_SAMP % FS_HZ == 0, "f0 must be bin-centred: f0*N/fs integer");
static_assert((long)FREF_HZ * N_SAMP % FS_HZ == 0, "fref must be bin-centred: fref*N/fs integer");

// ---------------- servo ----------------
constexpr int   SERVO_CENTER_DEG = 90;
constexpr int   SERVO_SIGN       = +1;     // +1: positive phi turns the horn left (CCW from above)
constexpr float PHI_MAX_DEG      = 80.0f;
constexpr float SLEW_DEG_S       = 300.0f; // servo speed limit [deg/s]; SG90 free-run is ~600

// ---------------- validity thresholds (re-applicable offline) ----------------
constexpr float A_MIN   = 3.0f;
constexpr float SNR_MIN = 4.0f;
constexpr int   CLIP_LO = 50, CLIP_HI = 4045;
constexpr int   LATE_US = 20;
constexpr int   LATE_MAX = N_SAMP / 20;

Servo servo;
float phi_c = 0.0f, dlt = 25.0f;
int   settle_ms = 40;              // ramped moves block until the horn arrives, so this only covers ringing
int   servo_now = SERVO_CENTER_DEG;
uint32_t seq = 0;

struct Cap { float A, Aref; uint16_t pp; bool clip; uint16_t late; };

// ---------------- output: USB always, Bluetooth while a client is connected ----------------
static void emit(const char* s) {
  Serial.print(s);
#if USE_BT
  if (SerialBT.hasClient()) SerialBT.print(s);
#endif
}

static void emitf(const char* fmt, ...) {
  char b[192];
  va_list a; va_start(a, fmt); vsnprintf(b, sizeof b, fmt, a); va_end(a);
  emit(b);
}

// ---------------- signal processing ----------------
static float goertzel(const int16_t* x, float mean, int k) {
  const float coeff = 2.0f * cosf(2.0f * PI * k / N_SAMP);
  float s1 = 0, s2 = 0;
  for (int i = 0; i < N_SAMP; i++) {
    float s = (x[i] - mean) + coeff * s1 - s2;
    s2 = s1; s1 = s;
  }
  float p = s1 * s1 + s2 * s2 - coeff * s1 * s2;
  return 2.0f * sqrtf(p > 0 ? p : 0) / N_SAMP;
}

static Cap capture() {
  static int16_t x[N_SAMP];
  Cap c{};
  int mn = 4095, mx = 0; long sum = 0;
  uint32_t t0 = micros();
  for (int i = 0; i < N_SAMP; i++) {
    uint32_t due = t0 + (uint32_t)(((uint64_t)i * 1000000ULL) / FS_HZ);
    while ((int32_t)(micros() - due) < 0) {}
    if ((int32_t)(micros() - due) > LATE_US) c.late++;
    int v = analogRead(PIN_MIC);
    x[i] = (int16_t)v; sum += v;
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  float mean = (float)sum / N_SAMP;
  c.A = goertzel(x, mean, K0);
  c.Aref = goertzel(x, mean, KREF);
  c.pp = (uint16_t)(mx - mn);
  c.clip = (mn <= CLIP_LO) || (mx >= CLIP_HI);
  return c;
}

static float moveTo(float phi) {                              // ramped; returns the actual (rounded) angle
  phi = constrain(phi, -PHI_MAX_DEG, PHI_MAX_DEG);
  int target = SERVO_CENTER_DEG + SERVO_SIGN * (int)lroundf(phi);
  int step = (target > servo_now) ? 1 : -1;
  while (servo_now != target) {
    servo_now += step;
    servo.write(servo_now);
    delayMicroseconds((uint32_t)(1e6f / SLEW_DEG_S));
  }
  return (float)(SERVO_SIGN * (target - SERVO_CENTER_DEG));
}

static uint32_t adcMicros() {
  uint32_t t = micros();
  for (int i = 0; i < 1000; i++) (void)analogRead(PIN_MIC);
  return (micros() - t) / 1000;
}

static void printConfig() {
  uint32_t us = adcMicros();
  emitf("C,%d,%d,%d,%d,%d,%lu\n", FS_HZ, N_SAMP, F0_HZ, FREF_HZ, settle_ms, (unsigned long)us);
  if (us * 1.15f > 1e6f / FS_HZ) emit("# WARN analogRead too slow for FS_HZ: lower FS_HZ (keep f*N/fs integer)\n");
}

// ---------------- commands from USB and Bluetooth ----------------
static void applyCmd(const char* s) {
  float v = atof(s + 1);
  switch (s[0]) {
    case 'P': phi_c = constrain(v, -PHI_MAX_DEG, PHI_MAX_DEG); break;
    case 'D': dlt = constrain(v, 1.0f, 60.0f); break;
    case 'S': settle_ms = constrain((int)v, 20, 1000); break;
    case '?': printConfig(); break;
  }
}

struct LineReader { char buf[24]; uint8_t n = 0; };

static void pollStream(Stream& s, LineReader& r) {
  while (s.available()) {
    char ch = (char)s.read();
    if (ch == '\n' || ch == '\r') { if (r.n) { r.buf[r.n] = 0; applyCmd(r.buf); r.n = 0; } }
    else if (r.n < sizeof(r.buf) - 1) r.buf[r.n++] = ch;
  }
}

static void handleSerial() {
  static LineReader usb;
  pollStream(Serial, usb);
#if USE_BT
  static LineReader bt;
  pollStream(SerialBT, bt);
#endif
}

void setup() {
  Serial.begin(115200);
#if USE_BT
  if (!SerialBT.begin(BT_NAME)) Serial.println("# WARN bluetooth failed to start");
#endif
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  ESP32PWM::allocateTimer(0);
  servo.setPeriodHertz(50);
  servo.attach(PIN_SERVO, 500, 2400);
  servo.write(SERVO_CENTER_DEG); delay(500);
#if USE_BT
  emitf("# esp32_acoustic_frontend_bt boot, bluetooth name %s\n", BT_NAME);
#else
  emit("# esp32_acoustic_frontend_bt boot, bluetooth disabled\n");
#endif
  printConfig();
}

void loop() {
  handleSerial();
  float pm = moveTo(phi_c - dlt);  delay(settle_ms);  Cap cm = capture();  uint32_t t_m = millis();
  float pp = moveTo(phi_c + dlt);  delay(settle_ms);  Cap cp = capture();  uint32_t t_p = millis();

  bool clip = cm.clip || cp.clip, late = cm.late > LATE_MAX || cp.late > LATE_MAX;
  bool valid = !clip && cm.A >= A_MIN && cp.A >= A_MIN &&
               cm.A >= SNR_MIN * cm.Aref && cp.A >= SNR_MIN * cp.Aref;
  unsigned flags = (valid ? 1u : 0u) | (clip ? 2u : 0u) | (late ? 4u : 0u);

  emitf("M,%lu,%lu,%lu,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%u,%u,%u\n",
        (unsigned long)seq++, (unsigned long)t_p, (unsigned long)(t_p - t_m),
        0.5f * (pm + pp), 0.5f * (pp - pm),
        cm.A, cp.A, cm.Aref, cp.Aref, (unsigned)cm.pp, (unsigned)cp.pp, flags);
}
