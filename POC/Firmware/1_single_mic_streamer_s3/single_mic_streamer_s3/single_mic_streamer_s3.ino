/*
 * Single-channel MAX4466 streamer for ESP32-S3 — delay-precision POC.
 *
 * Matches single_mic_precision_poc.py's parser: one raw ADC integer
 * (0-4095) per line, nothing else. Any non-numeric line is silently
 * dropped by the Python side, so don't add other Serial.print() calls
 * without accounting for that.
 *
 * HARDWARE:
 *   MAX4466 OUT -> ESP32-S3 ADC1 pin. Use an ADC1 channel, not ADC2 -
 *   ADC2 shares hardware with Wi-Fi and is unreliable when Wi-Fi is
 *   active. On most ESP32-S3 DevKit boards, GPIO1-GPIO10 are ADC1
 *   channels; this uses GPIO1 (ADC1_CH0). Check your specific board's
 *   pinout silkscreen/schematic before wiring - S3 GPIO-to-ADC1-channel
 *   mapping differs from classic ESP32.
 *   MAX4466 VCC -> 3.3V, GND -> GND.
 *
 * BAUD: 921600, matching the Python-side fix for the classic-ESP32 draft
 * (115200 could not sustain 10 kHz of framed integer samples without
 * drops - 10,000 samples/s x ~6 bytes/sample x 10 bits/byte handily
 * exceeds 115200 bps). Both sides of the serial link must agree on this.
 *
 * TIMING: paced by the Arduino-ESP32 core's hardware timer API
 * (esp32-hal-timer), which is supported on S3 and abstracts the
 * timer-group differences between classic ESP32 and S3 - avoids using
 * the lower-level driver/timer.h IDF calls, which have different
 * argument signatures on S3 across core versions.
 */

#define MIC_PIN 4               //  on most ESP32-S3 boards - verify for yours
const uint32_t SAMPLE_RATE_HZ = 10000;

volatile bool sample_ready = false;
volatile uint16_t latest_sample = 0;

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  latest_sample = analogRead(MIC_PIN);
  sample_ready = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  Serial.begin(921600);
  while (!Serial) { ; }  // S3 uses native USB-CDC; wait for host to open port

  analogReadResolution(12);        // 0-4095, matches ADC_MAX in Python
  analogSetAttenuation(ADC_11db);  // full ~0-3.3V input range

  // timerBegin signature on the Arduino-ESP32 core (S3-compatible):
  // frequency-based, not prescaler-based, as of core 3.x. If your
  // installed core is 2.x, use the legacy 4-arg form instead - see
  // fallback note below.
  timer = timerBegin(1000000);              // 1 MHz timer tick = 1 us resolution
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 1000000 / SAMPLE_RATE_HZ, true, 0);  // period in us, auto-reload
}

void loop() {
  if (sample_ready) {
    uint16_t val;
    portENTER_CRITICAL(&timerMux);
    val = latest_sample;
    sample_ready = false;
    portEXIT_CRITICAL(&timerMux);

    Serial.println(val);
  }
}

/*
 * FALLBACK - if your installed Arduino-ESP32 core is v2.x (pre-3.0), the
 * timer API signatures above will NOT compile (timerBegin/timerAlarm
 * signatures changed in core 3.0). Replace setup()'s timer block with:
 *
 *   timer = timerBegin(0, 80, true);              // timer 0, /80 prescaler -> 1us tick @ 80MHz APB
 *   timerAttachInterrupt(timer, &onTimer, true);
 *   timerAlarmWrite(timer, 1000000 / SAMPLE_RATE_HZ, true);
 *   timerAlarmEnable(timer);
 *
 * Run `arduino-cli core list` or check Tools > Board Manager in the IDE
 * to confirm your "esp32" core package version if unsure which applies.
 */
