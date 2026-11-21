/*
 * Dual-MAX4466 Time-Difference-of-Arrival (TDOA) bearing estimator.
 *
 * HARDWARE ASSUMPTION:
 *   Two MAX4466 analog outputs wired to two ADC1 channels on an ESP32
 *   classic (e.g. GPIO34 = ADC1_CH6, GPIO35 = ADC1_CH7). ADC1 is required
 *   (not ADC2 - it conflicts with Wi-Fi). Both channels are read via the
 *   continuous-mode DMA ADC driver, which hardware-sequences the two
 *   channels back-to-back at a fixed, deterministic sample rate. This is
 *   the actual synchronization primitive here - NOT I2S (I2S is for
 *   digital MEMS mics like the INMP441, not analog electret breakouts).
 *
 * WHY NOT sequential analogRead() calls:
 *   Two separate analogRead(PIN_A); analogRead(PIN_B) calls incur ~10-20us
 *   of inter-channel skew from ADC mux switching + conversion time. At a
 *   10cm mic baseline, full-scale ITD is only ~292us, so 10-20us of
 *   UNCONTROLLED skew is 3-7% of your entire dynamic range, and it is not
 *   constant - it depends on code path timing. Continuous-mode DMA with a
 *   fixed channel sequence removes this by design: samples are pulled by
 *   a hardware timer, not by loop() timing.
 *
 * SAMPLE RATE:
 *   48 kHz per channel (96 kHz aggregate across 2 channels) puts ~14
 *   samples across the full ITD swing at a 10cm baseline - enough for
 *   parabolic sub-sample interpolation of the cross-correlation peak.
 *
 * OUTPUT (over USB serial, ASCII, one line per estimate):
 *   theta_deg,delta_t_us,peak_corr,confidence,rms1_mv,rms2_mv
 *
 *   theta_deg   : estimated bearing, 0 = broadside toward mic1, sign
 *                 convention: positive = source closer to mic1.
 *                 NOTE: a single mic pair has a front-back ambiguity -
 *                 theta and (180 - theta) are indistinguishable. Resolve
 *                 this mechanically (e.g. baffle/shield behind the pair)
 *                 or by combining with the rotating-arm sweep.
 *   delta_t_us  : raw estimated inter-channel delay, sub-sample
 *                 interpolated, signed.
 *   peak_corr   : normalized cross-correlation peak value [0,1] -
 *                 low values mean the "delay" is noise, not signal.
 *   confidence  : peak_corr re-expressed 0-100 for convenience.
 *   rms1_mv,rms2_mv : per-channel RMS AC amplitude, for your existing
 *                 distance-proxy logic (unchanged from single-mic case).
 */

#include <driver/adc.h>
#include <esp_adc_cal.h>

// ---- Configuration ----
#define MIC1_ADC_CHANNEL ADC1_CHANNEL_6  // GPIO34
#define MIC2_ADC_CHANNEL ADC1_CHANNEL_7  // GPIO35

const uint32_t SAMPLE_RATE_HZ = 48000;      // per channel
const size_t   WINDOW_SAMPLES = 512;        // ~10.7 ms window per estimate
const float    MIC_BASELINE_M = 0.10f;      // measured mic-to-mic distance
const float    SPEED_OF_SOUND = 343.0f;     // m/s, adjust for temperature if needed
const float    VREF_MV = 3300.0f;
const float    ADC_MAX_COUNTS = 4095.0f;
const float    BIAS_COUNTS = 2048.0f;

// Max physically possible |delay| in samples (anything beyond this in the
// correlation search is nonphysical and indicates noise, not a real delay).
const int MAX_LAG_SAMPLES = (int)ceil((MIC_BASELINE_M / SPEED_OF_SOUND) * SAMPLE_RATE_HZ) + 1;

int16_t buf1[WINDOW_SAMPLES];
int16_t buf2[WINDOW_SAMPLES];

// ---- Dual-channel continuous ADC acquisition ----
// Uses the ADC continuous (DMA) driver so both channels are pulled on a
// single hardware-timed sequence rather than software-timed loop() calls.
#include <driver/adc_continuous_impl.h> // internal but stable across IDF 4.x
#include <driver/adc_types.h>

adc_continuous_handle_t adc_handle = nullptr;

void setup_dual_adc() {
  adc_continuous_handle_cfg_t adc_config = {
    .max_store_buf_size = 4096,
    .conv_frame_size = 256,
  };
  adc_continuous_new_handle(&adc_config, &adc_handle);

  adc_digi_pattern_config_t adc_pattern[2] = {};
  adc_pattern[0].atten = ADC_ATTEN_DB_11;
  adc_pattern[0].channel = MIC1_ADC_CHANNEL;
  adc_pattern[0].unit = ADC_UNIT_1;
  adc_pattern[0].bit_width = ADC_BITWIDTH_12;

  adc_pattern[1].atten = ADC_ATTEN_DB_11;
  adc_pattern[1].channel = MIC2_ADC_CHANNEL;
  adc_pattern[1].unit = ADC_UNIT_1;
  adc_pattern[1].bit_width = ADC_BITWIDTH_12;

  adc_continuous_config_t dig_cfg = {
    .pattern_num = 2,
    .adc_pattern = adc_pattern,
    .sample_freq_hz = SAMPLE_RATE_HZ * 2,  // hardware alternates ch1/ch2, so
                                            // the raw conversion rate must be
                                            // 2x the per-channel rate you want
    .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
  };
  adc_continuous_config(adc_handle, &dig_cfg);
  adc_continuous_start(adc_handle);
}

// Reads WINDOW_SAMPLES samples for EACH channel, de-interleaving the
// hardware-sequenced ch1/ch2/ch1/ch2... stream into two separate buffers.
// Returns true if a full window was captured before timing out.
bool capture_window() {
  uint8_t raw_buf[WINDOW_SAMPLES * 2 * sizeof(adc_digi_output_data_t)];
  uint32_t bytes_read = 0;
  size_t idx1 = 0, idx2 = 0;

  uint32_t timeout_ms = 100;
  uint32_t start = millis();

  while ((idx1 < WINDOW_SAMPLES || idx2 < WINDOW_SAMPLES) &&
         (millis() - start < timeout_ms)) {
    esp_err_t ret = adc_continuous_read(adc_handle, raw_buf, sizeof(raw_buf), &bytes_read, 0);
    if (ret != ESP_OK) continue;

    for (int i = 0; i < bytes_read; i += sizeof(adc_digi_output_data_t)) {
      adc_digi_output_data_t *p = (adc_digi_output_data_t*)&raw_buf[i];
      uint32_t chan = p->type2.channel;
      uint32_t val = p->type2.data;

      if (chan == MIC1_ADC_CHANNEL && idx1 < WINDOW_SAMPLES) {
        buf1[idx1++] = (int16_t)val;
      } else if (chan == MIC2_ADC_CHANNEL && idx2 < WINDOW_SAMPLES) {
        buf2[idx2++] = (int16_t)val;
      }
    }
  }
  return (idx1 == WINDOW_SAMPLES && idx2 == WINDOW_SAMPLES);
}

// ---- Cross-correlation with sub-sample (parabolic) interpolation ----
// tau_hat = argmax_tau sum_n x1(n) x2(n+tau), searched only over
// physically-possible lags (+/- MAX_LAG_SAMPLES), then refined with a
// 3-point parabolic fit around the discrete peak for sub-sample precision.
struct CorrResult {
  float delta_t_us;
  float peak_corr_normalized;
};

CorrResult cross_correlate(int16_t *x1, int16_t *x2, size_t n) {
  // Remove DC bias (per-window mean) before correlating - the fixed
  // 2048-count bias would otherwise dominate the correlation sum and mask
  // the actual AC correlation structure.
  float mean1 = 0, mean2 = 0;
  for (size_t i = 0; i < n; i++) { mean1 += x1[i]; mean2 += x2[i]; }
  mean1 /= n; mean2 /= n;

  float energy1 = 0, energy2 = 0;
  for (size_t i = 0; i < n; i++) {
    energy1 += (x1[i] - mean1) * (x1[i] - mean1);
    energy2 += (x2[i] - mean2) * (x2[i] - mean2);
  }
  float norm = sqrt(energy1 * energy2);
  if (norm < 1e-6f) {
    return { 0.0f, 0.0f };  // silence - no meaningful correlation possible
  }

  int best_lag = 0;
  float best_val = -1e18f;
  float corr_vals[2 * MAX_LAG_SAMPLES + 1];

  for (int lag = -MAX_LAG_SAMPLES; lag <= MAX_LAG_SAMPLES; lag++) {
    float sum = 0;
    size_t count = 0;
    for (size_t i = 0; i < n; i++) {
      long j = (long)i + lag;
      if (j < 0 || j >= (long)n) continue;
      sum += (x1[i] - mean1) * (x2[j] - mean2);
      count++;
    }
    float val = sum;  // unnormalized within-window; normalize below for reporting
    corr_vals[lag + MAX_LAG_SAMPLES] = val;
    if (val > best_val) {
      best_val = val;
      best_lag = lag;
    }
  }

  // Parabolic interpolation around best_lag using its neighbors, guarding
  // against the peak sitting at the edge of the search range (in which
  // case interpolation is invalid and we report the raw integer lag - this
  // also signals the true delay may exceed the physical max, i.e. a
  // spurious/noise correlation).
  float tau_refined = (float)best_lag;
  int center_idx = best_lag + MAX_LAG_SAMPLES;
  if (best_lag > -MAX_LAG_SAMPLES && best_lag < MAX_LAG_SAMPLES) {
    float y0 = corr_vals[center_idx - 1];
    float y1 = corr_vals[center_idx];
    float y2 = corr_vals[center_idx + 1];
    float denom = (y0 - 2.0f * y1 + y2);
    if (fabs(denom) > 1e-9f) {
      float offset = 0.5f * (y0 - y2) / denom;
      // Clamp - a well-formed parabola near a real peak gives |offset|<1;
      // anything larger means the local shape isn't parabolic (noise).
      if (fabs(offset) < 1.0f) {
        tau_refined = (float)best_lag + offset;
      }
    }
  }

  float delta_t_us = (tau_refined / (float)SAMPLE_RATE_HZ) * 1e6f;
  float peak_corr_normalized = best_val / norm;

  return { delta_t_us, peak_corr_normalized };
}

float compute_rms_mv(int16_t *x, size_t n) {
  float sum_sq = 0;
  for (size_t i = 0; i < n; i++) {
    float ac = ((float)x[i] - BIAS_COUNTS) / ADC_MAX_COUNTS * VREF_MV;
    sum_sq += ac * ac;
  }
  return sqrt(sum_sq / n);
}

void setup() {
  Serial.begin(921600);  // high baud: needed since we may stream more than
                          // single scalars if debugging; estimates alone
                          // are lightweight so this is headroom, not a
                          // requirement for the scalar-only output mode.
  delay(200);
  setup_dual_adc();
  Serial.println("theta_deg,delta_t_us,peak_corr,confidence,rms1_mv,rms2_mv");
}

void loop() {
  if (!capture_window()) {
    Serial.println("NAN,NAN,0.0,0,NAN,NAN");  // capture timeout - upstream should discard this row
    return;
  }

  CorrResult r = cross_correlate(buf1, buf2, WINDOW_SAMPLES);

  // Convert delay to bearing. Clamp the argument to asin() to [-1,1]
  // since sensor/timing noise can push |delta_t| slightly past the
  // physical max, which would otherwise NaN the asin().
  float max_delta_t_us = (MIC_BASELINE_M / SPEED_OF_SOUND) * 1e6f;
  float sin_theta = r.delta_t_us / max_delta_t_us;
  if (sin_theta > 1.0f) sin_theta = 1.0f;
  if (sin_theta < -1.0f) sin_theta = -1.0f;
  float theta_deg = asin(sin_theta) * 180.0f / PI;

  float rms1 = compute_rms_mv(buf1, WINDOW_SAMPLES);
  float rms2 = compute_rms_mv(buf2, WINDOW_SAMPLES);

  float confidence = r.peak_corr_normalized * 100.0f;
  if (confidence < 0) confidence = 0;

  Serial.printf("%.2f,%.2f,%.4f,%.1f,%.2f,%.2f\n",
                theta_deg, r.delta_t_us, r.peak_corr_normalized, confidence, rms1, rms2);
}
