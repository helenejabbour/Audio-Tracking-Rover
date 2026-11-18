/*
  esp32_acoustic_frontend.ino
  ESP32 + MAX4466 + SG90 horn: paired Goertzel amplitudes A- / A+ at phi_c -/+ delta.

  Amplitude (peak, ADC counts):  A = (2/N) | sum (x[n]-mean) e^{-j 2 pi f n / fs} |     (report Eq. goertzel)
  f0 and fref are bin-centred (f*N/fs integer) so there is no leakage; enforced by static_assert.
  fref is an off-tone bin used as a noise-floor / SNR reference.

  Serial, 115200, ASCII lines
    ESP32 -> host
      C,<fs>,<N>,<f0>,<fref>,<settle_ms>,<adc_us>,<phi_c>,<dlt>   config (at boot and on '?'); adc_us = analogRead() duration;
                                                                 phi_c, dlt = COMMANDED centre / dither now in force
      M,<seq>,<t_ms>,<dt_pair_ms>,<phi_c>,<dlt>,<A_m>,<A_p>,<Aref_m>,<Aref_p>,<pp_m>,<pp_p>,<flags>
         t_ms       millis() at end of the second capture of the pair
         dt_pair_ms time between the ends of the two captures
         phi_c,dlt  ACTUAL arm centre / dither after servo rounding to 1 deg [deg, + = counter-clockwise]
         A_m, A_p   always the amplitude at phi_c - dlt and phi_c + dlt, whichever was captured first
         pp_*       peak-to-peak ADC counts of each capture (use it to set the MAX4466 gain)
         flags      bit0 valid, bit1 clipped, bit2 timing overrun (>5% of samples started late),
                    bit3 order: 0 = A- captured first, 1 = A+ captured first (alternates every packet)
    host -> ESP32:  P<deg> arm centre | D<deg> dither | S<ms> servo settle | ?  print config

  v2 (Oct 28): servo jumps to the target and waits |move|/SLEW_DEG_S + settle (the 1-deg ramp took ~40 ms/deg);
               capture order alternates every packet so an order-dependent disturbance averages out of ln(A+/A-);
               default dither 30 deg; C line reports phi_c and dlt so the host knows when its commands took effect.
*/
#include <Arduino.h>
#include <ESP32Servo.h>

// ---------------- pins (edit for your board) ----------------
constexpr int PIN_MIC   = 34;    // MAX4466 OUT
constexpr int PIN_SERVO = 32;    // SG90 signal

// ---------------- acquisition ----------------
constexpr int FS_HZ   = 16000;   // sampling rate
constexpr int N_SAMP  = 512;     // window length (32 ms)
constexpr int F0_HZ   = 4000;    // beacon tone: use the tone of your proof of concept if different
constexpr int FREF_HZ = 3000;    // noise reference, must not be a harmonic of f0
constexpr int K0   = (int)((long)F0_HZ   * N_SAMP / FS_HZ);
constexpr int KREF = (int)((long)FREF_HZ * N_SAMP / FS_HZ);
static_assert((long)F0_HZ   * N_SAMP % FS_HZ == 0, "f0 must be bin-centred: f0*N/fs integer");
static_assert((long)FREF_HZ * N_SAMP % FS_HZ == 0, "fref must be bin-centred: fref*N/fs integer");
constexpr float SLEW_DEG_S = 300.0f;   // assumed servo speed under load [deg/s], used to wait out a move; SG90 free-run is ~600

// ---------------- servo ----------------
constexpr int   SERVO_CENTER_DEG = 110;     // servo angle at which the horn points straight ahead
constexpr int   SERVO_SIGN       = +1;     // +1: positive phi turns the horn left (CCW from above); flip if reversed
constexpr float PHI_MAX_DEG      = 80.0f;  // keep inside mechanical limits of your mount

// ---------------- validity thresholds (re-applicable offline from the raw values) ----------------
constexpr float A_MIN   = 3.0f;    // counts
constexpr float SNR_MIN = 4.0f;    // A / Aref
constexpr int   CLIP_LO = 50, CLIP_HI = 4045;   // 12-bit ADC rails
constexpr int   LATE_US = 20;      // a sample started this late counts as late
constexpr int   LATE_MAX = N_SAMP / 20;   // more late samples than this => overrun flag

Servo servo;
float phi_c = 0.0f, dlt = 30.0f;   // dither: larger = better bearing precision but slower pairs; choose after E1/E4
int   settle_ms = 40;             // servo dwell before each capture, after the move itself
uint32_t seq = 0;
bool  plus_first = false;         // capture order, toggled every packet

struct Cap { float A, Aref; uint16_t pp; bool clip; uint16_t late; };

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
    while ((int32_t)(micros() - due) < 0) {}                 // paced busy-wait
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

static int servo_now = SERVO_CENTER_DEG;

static float moveTo(float phi) {                              // one jump, then wait out the travel; returns the actual (rounded) angle
  phi = constrain(phi, -PHI_MAX_DEG, PHI_MAX_DEG);
  int target = SERVO_CENTER_DEG + SERVO_SIGN * (int)lroundf(phi);
  int travel = abs(target - servo_now);
  if (travel > 0) {
    servo.write(target);
    delay((uint32_t)(1000.0f * travel / SLEW_DEG_S + 0.5f));
    servo_now = target;
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
  Serial.printf("C,%d,%d,%d,%d,%d,%lu,%.1f,%.1f\n", FS_HZ, N_SAMP, F0_HZ, FREF_HZ, settle_ms, (unsigned long)us, phi_c, dlt);
  if (us * 1.15f > 1e6f / FS_HZ) Serial.println("# WARN analogRead too slow for FS_HZ: lower FS_HZ (keep f*N/fs integer)");
}

static void applyCmd(const char* s) {
  float v = atof(s + 1);
  switch (s[0]) {
    case 'P': phi_c = constrain(v, -PHI_MAX_DEG, PHI_MAX_DEG); break;
    case 'D': dlt = constrain(v, 1.0f, 60.0f); break;
    case 'S': settle_ms = constrain((int)v, 20, 1000); break;
    case '?': printConfig(); break;
  }
}

static void handleSerial() {
  static char buf[24]; static uint8_t n = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') { if (n) { buf[n] = 0; applyCmd(buf); n = 0; } }
    else if (n < sizeof(buf) - 1) buf[n++] = ch;
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);                             // ~0-3.1 V; MAX4466 idles at Vcc/2
  ESP32PWM::allocateTimer(0);
  servo.setPeriodHertz(50);
  servo.attach(PIN_SERVO, 500, 2400);
  servo.write(SERVO_CENTER_DEG); delay(500);
  Serial.println("# esp32_acoustic_frontend boot");
  printConfig();
}

void loop() {
  handleSerial();
  // Alternate which side is captured first, so anything that depends on capture ORDER (a drifting source,
  // first vs second capture) cancels in the average of ln(A+/A-). The servo just ping-pongs, so each side is
  // still always approached from the other side: a SIDE/direction effect (cable pull, horn asymmetry) does not
  // cancel, but comparing packets by flags bit3 tells the two apart (order effect flips sign, side effect does not).
  const float s1 = plus_first ? +1.0f : -1.0f;
  float ph1 = moveTo(phi_c + s1 * dlt);  delay(settle_ms);  Cap c1 = capture();  uint32_t t1 = millis();
  float ph2 = moveTo(phi_c - s1 * dlt);  delay(settle_ms);  Cap c2 = capture();  uint32_t t2 = millis();
  const bool pf = plus_first;
  plus_first = !plus_first;

  const Cap&  cm = pf ? c2 : c1;   // capture at phi_c - dlt
  const Cap&  cp = pf ? c1 : c2;   // capture at phi_c + dlt
  const float pm = pf ? ph2 : ph1;
  const float pp = pf ? ph1 : ph2;

  bool clip = cm.clip || cp.clip, late = cm.late > LATE_MAX || cp.late > LATE_MAX;
  bool valid = !clip && cm.A >= A_MIN && cp.A >= A_MIN &&
               cm.A >= SNR_MIN * cm.Aref && cp.A >= SNR_MIN * cp.Aref;
  unsigned flags = (valid ? 1u : 0u) | (clip ? 2u : 0u) | (late ? 4u : 0u) | (pf ? 8u : 0u);

  Serial.printf("M,%lu,%lu,%lu,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%u,%u,%u\n",
                (unsigned long)seq++, (unsigned long)t2, (unsigned long)(t2 - t1),
                0.5f * (pm + pp), 0.5f * (pp - pm),
                cm.A, cp.A, cm.Aref, cp.Aref, (unsigned)cm.pp, (unsigned)cp.pp, flags);
}
