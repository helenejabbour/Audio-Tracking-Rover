/* Acoustic frontend V2. Classic ESP32, Arduino-ESP32 3.x, USB 115200.
 * No external servo library. GPIO34=ADC1 input, GPIO32=horn PWM.
 * Continuous multi-tone beacon (frequency diversity, Phase B step 3, 20 Nov 2026);
 * paired captures have separate midpoint timestamps. Per pair, two lines with the same seq:
 * B2,seq,M,a-_1..a-_M,a+_1..a+_M,Nm,Np*CRC16   per-tone amplitudes (raw, not noise-subtracted)
 * R2,seq,tm_us,tp_us,emit_us,phi_m,phi_p,Am,Ap,Nm,Np,pp_m,pp_p,flags*CRC16
 * B2 is sent first, so a host can recombine the bins (rover.sensing.combine_bins) before
 * it handles the R2 line. Hosts that only know R2 ignore B2 and use the firmware combination.
 *   Am/Ap = sqrt(max(0, mean_k a_k^2 - N^2)): equal-weight power average over the M tone
 *           bins minus the in-band noise power. Coherent multipath fringes have a different
 *           phase in each bin, so averaging power over frequency removes most of the
 *           position dependence of a single tone (fringe spacing in frequency = c/dL).
 *   Nm/Np = sqrt(mean noise-bin power); the noise bins sit midway between the tones.
 *   wide16 includes 4000 Hz (the old single tone); horn12 avoids every 1 kHz multiple.
 * BEACON_PROFILE selects the tone set: 2 = horn12 (default, 3.19-4.56 kHz), 1 = wide16 (2.5-6.25 kHz).
 * Legacy single-tone behaviour: TONE_K={128}, NOISE_K={96} (4 kHz tone, 3 kHz reference),
 * USE_HANN=0. A is then noise-subtracted, which the old sketch did not do.
 * CRC16-CCITT-FALSE: ASCII before '*', init FFFF, polynomial 1021.
 *   tm_us/tp_us = midpoints of the A- / A+ captures; either may come first (bit 3).
 * flags: 1 valid, 2 ADC clipping, 4 sample timing failure; valid excludes 2/4.
 *        8 capture order: 0 = A- captured first, 1 = A+ captured first (alternates every pair).
 * Commands P<deg>, D<deg>, S<ms>, ?; one newline-terminated command per line.
 *
 * Servo constants are the wired-v2 values (esp32_acoustic_wired.ino, Oct 28):
 * centre 110 deg, +1 = left, 500-2400 us (the ESP32Servo attach range), phi limit 80 deg.
 * SERVO_CENTER IS A PER-MOUNT CALIBRATION: 110 deg is where the horn pointed straight
 * ahead with the horn-to-servo mounting used for the Oct-Nov data (the horn is not
 * aligned 1:1 with the servo spline). Re-measure it whenever the horn, servo or bracket
 * is moved or reattached, and note the value in each session's notes.
 * HORN_LIMIT=80 is only an upper bound; experiments set dither separately (D, max 60).
 * With centre 110 the servo end stop (180 deg) limits phi to +70 on the left, so the
 * physical span is [-80, +70] deg. This only matters for |phi_c|+delta > 70 (the E1
 * sweep edges); E2-E4 never exceeded 60. This sketch clamps explicitly and reports the
 * true angle. If the re-measured centre changes, PHI_LO/PHI_HI follow automatically;
 * update horn_limit_left_deg in Config/measured_sensor.json to match.
 * Capture order alternates every pair, as in wired v2: consecutive pairs ping-pong, so
 * there is one 2*delta move per pair, and an order-dependent bias (target motion during
 * the ~0.4 s gap) averages out of ln(A+/A-).
 * Verify the servo constants with the horn disconnected before mounting it.
 *
 * Wireless (optional, -DUSE_BT=1; default is USB only): Bluetooth Classic SPP alongside USB. Every output line
 * goes to USB and, while a client is connected, to Bluetooth; commands are accepted from
 * either link. The line protocol is identical, so the host only changes its port
 * (Pi: /dev/rfcomm0 after `rfcomm bind`; Windows: the outgoing "Standard Serial over
 * Bluetooth link" COM port). Pairs emitted with no client connected are lost (seq gaps).
 * A line never blocks the sampling loop: if the BT transmit buffer cannot take the
 * whole line, it is dropped and counted (bt_drop in the C2 line).
 * Requires a classic ESP32 (not S2/S3/C3/C6): BLE-only chips have no SPP.
 * Default build (USE_BT=0) is USB only, the deployment path.
 */
#include <Arduino.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#ifndef USE_BT
#define USE_BT 0   // deployment link is USB serial to the follower Pi; -DUSE_BT=1 enables SPP
#endif
#if USE_BT
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Classic Bluetooth is not enabled: use a classic ESP32 with board 'ESP32 Dev Module', or build with -DUSE_BT=0"
#endif
BluetoothSerial SerialBT;
constexpr const char *BT_NAME="AcousticFollower";   // name shown when pairing
uint32_t bt_dropped=0;
bool bt_was_connected=false;
#endif
// Declared here so Arduino's generated prototypes can see it.
struct LineReader { char buf[32]; uint8_t n=0; bool overflow=false; };

constexpr int MIC_PIN=34, SERVO_PIN=32;
constexpr int FS=16000, NS=512;
// Bin k is k*FS/NS = k*31.25 Hz. The beacon WAV must contain exactly the TONE_K tones.
#ifndef BEACON_PROFILE
#define BEACON_PROFILE 2
#endif
#if BEACON_PROFILE==2
// "horn12" (default, 20 Nov s1 findings): the horn has bearing slope only at ~3.25-4.5 kHz,
// and USB-frame interference sits on every 1 kHz multiple. 12 tones, 3187.5..4562.5 Hz every
// 125 Hz (k = 102..146 step 4), none within one bin of a 1 kHz multiple (k % 32 in {31,0,1}).
// Noise bins midway (k+-2), except k = 128 (4000 Hz, interference line).
// Beacon: Tools/make_tone.py --spec Data/stimuli/horn12_spec.json
constexpr const char *PROFILE_NAME="horn12";
constexpr int TONE_K[]={102,106,110,114,118,122,126,130,134,138,142,146};
constexpr int NOISE_K[]={100,104,108,112,116,120,124,132,136,140,144,148};
constexpr bool AVOID_KHZ=true;
#else
// "wide16" (first multi-tone session s1): 2500..6250 Hz every 250 Hz (k = 80..200 step 8),
// noise bins midway (k+4). Beacon: Tools/make_tone.py --multitone.
constexpr const char *PROFILE_NAME="wide16";
constexpr int TONE_K[]={80,88,96,104,112,120,128,136,144,152,160,168,176,184,192,200};
constexpr int NOISE_K[]={84,92,100,108,116,124,132,140,148,156,164,172,180,188,196,204};
constexpr bool AVOID_KHZ=false;
#endif
constexpr int M_TONES=sizeof(TONE_K)/sizeof(TONE_K[0]), M_NOISE=sizeof(NOISE_K)/sizeof(NOISE_K[0]);
#ifndef USE_HANN
#define USE_HANN 1   // Hann window: no leakage from bin-centred tones into the noise bins 4 bins away
#endif
constexpr int SERVO_CENTER=110, SERVO_SIGN=1, SERVO_MIN=30, SERVO_MAX=180;
constexpr float HORN_LIMIT=80.f, SLEW_DEG_S=300.f;
constexpr int PULSE_MIN_US=500, PULSE_MAX_US=2400;
// Physical phi span once the servo range applies: [-80, +70] deg for the values above.
constexpr float PHI_LO=(SERVO_SIGN>0?SERVO_MIN-SERVO_CENTER:SERVO_CENTER-SERVO_MAX)<-HORN_LIMIT?-HORN_LIMIT:
                       (float)(SERVO_SIGN>0?SERVO_MIN-SERVO_CENTER:SERVO_CENTER-SERVO_MAX);
constexpr float PHI_HI=(SERVO_SIGN>0?SERVO_MAX-SERVO_CENTER:SERVO_CENTER-SERVO_MIN)>HORN_LIMIT?HORN_LIMIT:
                       (float)(SERVO_SIGN>0?SERVO_MAX-SERVO_CENTER:SERVO_CENTER-SERVO_MIN);
// SNR test on the band-combined, noise-subtracted amplitude. With 16 noise bins the noise
// power estimate has ~25 % relative sd, so silence gives A/N ~ 0.6 at 1 sigma; 2 is ~11 sigma.
// The old per-capture single-bin test (4) rejected captures that fell into a fringe null,
// which biased the surviving ratio; the combined test does not depend on one bin.
constexpr float A_MIN=3.f, SNR_MIN=(M_TONES>1?2.f:4.f);
// Member function, so Arduino's prototype generator does not insert prototypes above Capture.
struct BinCheck { static constexpr bool ok() {
  for(int i=0;i<M_TONES;i++){ if(TONE_K[i]<=0||TONE_K[i]>=NS/2) return false;
    for(int j=0;j<M_NOISE;j++) if(NOISE_K[j]==TONE_K[i]) return false; }
  for(int j=0;j<M_NOISE;j++) if(NOISE_K[j]<=0||NOISE_K[j]>=NS/2) return false;
  if(AVOID_KHZ) {   // 1 kHz = 32 bins; Hann leaks a bin-centred line into +-1 bin only
    for(int i=0;i<M_TONES;i++){int r=TONE_K[i]%32; if(r==31||r==0||r==1) return false;}
    for(int j=0;j<M_NOISE;j++){int r=NOISE_K[j]%32; if(r==31||r==0||r==1) return false;}
  }
  return true;
} };
static_assert(M_TONES>=1 && M_NOISE>=1 && BinCheck::ok(), "tone/noise bins must be distinct and below Nyquist");
static_assert(SERVO_CENTER>=SERVO_MIN && SERVO_CENTER<=SERVO_MAX, "invalid center");
static_assert(SERVO_MIN>=0 && SERVO_MAX<=180, "servo range is 0-180 deg");
static_assert(PHI_LO<0 && PHI_HI>0, "horn span must straddle boresight");
float center=0.f, delta=50.f;   // chosen dither (Matlab/horn_model_parameters.txt)
int settle_ms=40, servo_angle=SERVO_CENTER;
uint32_t seq=0, ready_ms=0;
float pair_pm=0.f, pair_pp=0.f, pair_second=0.f;
bool measuring_second=false, plus_first=false, failed=false;
struct Capture { float A,N; float tone[M_TONES]; uint16_t pp; uint32_t midpoint; bool clip,late; } c1;
float window_w[NS], window_sum=NS;

uint16_t crc16(const char *s) {
  uint16_t crc=0xffff;
  while (*s) { crc ^= (uint16_t)(uint8_t)*s++ << 8;
    for (int i=0;i<8;i++) crc=(crc&0x8000)?(uint16_t)((crc<<1)^0x1021):(uint16_t)(crc<<1);
  }
  return crc;
}

void servoWrite(int angle) {
  angle=constrain(angle,SERVO_MIN,SERVO_MAX);
  uint32_t pulse=PULSE_MIN_US+(uint32_t)(PULSE_MAX_US-PULSE_MIN_US)*angle/180;
  if (!ledcWrite(SERVO_PIN,(uint32_t)((uint64_t)pulse*65535/20000))) failed=true;
}

float moveHorn(float phi) {
  phi=constrain(phi,PHI_LO,PHI_HI);
  int angle=constrain(SERVO_CENTER+SERVO_SIGN*(int)lroundf(phi),SERVO_MIN,SERVO_MAX);
  int travel=abs(angle-servo_angle);
  servoWrite(angle); servo_angle=angle;
  ready_ms=millis()+(uint32_t)ceilf(1000.f*travel/SLEW_DEG_S)+settle_ms;
  return (float)(SERVO_SIGN*(angle-SERVO_CENTER));
}

void initWindow() {
  window_sum=0;
  for(int i=0;i<NS;i++){window_w[i]=USE_HANN?.5f-.5f*cosf(2*PI*i/NS):1.f;window_sum+=window_w[i];}
}

// Amplitude of a bin-centred sinusoid: 2|X_k|/sum(w), so a tone reads the same with or
// without the window (and the same as the old rectangular 4 kHz value).
float binAmplitude(const float *x,int k) {
  float coeff=2*cosf(2*PI*k/NS),s1=0,s2=0;
  for(int i=0;i<NS;i++){float s=x[i]+coeff*s1-s2;s2=s1;s1=s;}
  return 2*sqrtf(fmaxf(0,s1*s1+s2*s2-coeff*s1*s2))/window_sum;
}

Capture capture() {
  static int16_t x[NS]; static float xf[NS]; long sum=0; int mn=4095,mx=0,late=0;
  uint32_t start=micros(),first=0,last=0;
  for(int i=0;i<NS;i++) {
    uint32_t due=start+(uint32_t)((uint64_t)i*1000000/FS);
    while((int32_t)(micros()-due)<0){}
    uint32_t sample=micros();
    if((int32_t)(sample-due)>20)late++;
    int value=analogRead(MIC_PIN);
    if(i==0)first=sample; last=sample;
    x[i]=value;sum+=value;mn=min(mn,value);mx=max(mx,value);
  }
  float actual_fs=(NS-1)*1e6f/(uint32_t)(last-first);
  Capture c;
  const float mean=(float)sum/NS;
  for(int i=0;i<NS;i++)xf[i]=(x[i]-mean)*window_w[i];
  float pt=0,pn=0;
  for(int m=0;m<M_TONES;m++){c.tone[m]=binAmplitude(xf,TONE_K[m]);pt+=c.tone[m]*c.tone[m];}
  for(int j=0;j<M_NOISE;j++){float a=binAmplitude(xf,NOISE_K[j]);pn+=a*a;}
  pt/=M_TONES;pn/=M_NOISE;
  c.A=sqrtf(fmaxf(0,pt-pn));c.N=sqrtf(pn);
  c.pp=mx-mn;c.midpoint=first+(uint32_t)(last-first)/2;
  c.clip=mn<=50||mx>=4045;
  c.late=late>NS/20||fabsf(actual_fs/FS-1)>0.02f;
  return c;
}

// Output: USB always; Bluetooth while a client is connected and its buffer has room.
void emit(const char *line) {
  Serial.print(line);
#if USE_BT
  if(SerialBT.hasClient()) {
    if(SerialBT.availableForWrite()>=(int)strlen(line))SerialBT.print(line);
    else bt_dropped++;
  }
#endif
}

void emitf(const char *fmt,...) __attribute__((format(printf,1,2)));
void emitf(const char *fmt,...) {
  char b[600];va_list a;va_start(a,fmt);vsnprintf(b,sizeof(b),fmt,a);va_end(a);emit(b);
}

void config() {
#if USE_BT
  const char *link=SerialBT.hasClient()?"usb+bt":"usb";unsigned long drops=bt_dropped;
#else
  const char *link="usb";unsigned long drops=0;
#endif
  emitf("C2,profile=%s,fs=%d,N=%d,tones=%d,tone_k=%d:%d,noise_bins=%d,noise_k=%d:%d,window=%s,combine=power_nsub,snr_min=%.1f,"
        "settle_ms=%d,center=%d,sign=%d,limits=%d:%d,phi=%.1f:%.1f,phi_c=%.1f,dlt=%.1f,alternate=1,link=%s,bt_drop=%lu\n",
    PROFILE_NAME,FS,NS,M_TONES,TONE_K[0],TONE_K[M_TONES-1],M_NOISE,NOISE_K[0],NOISE_K[M_NOISE-1],USE_HANN?"hann":"rect",SNR_MIN,
    settle_ms,SERVO_CENTER,SERVO_SIGN,SERVO_MIN,SERVO_MAX,PHI_LO,PHI_HI,center,delta,link,drops);
}

void command(const char *s) {
  if(s[0]=='?' && s[1]==0){config();return;}
  char *end;float value=strtof(s+1,&end);
  if(end==s+1||*end!=0||!isfinite(value)){emit("# invalid command\n");return;}
  switch(s[0]) {
    case 'P':center=constrain(value,PHI_LO+delta,PHI_HI-delta);break;
    case 'D':delta=constrain(value,1.f,60.f);center=constrain(center,PHI_LO+delta,PHI_HI-delta);break;
    case 'S':settle_ms=constrain((int)value,20,1000);break;
    default:emit("# unknown command\n");break;
  }
}

void pollStream(Stream &in,LineReader &r) {
  while(in.available()) {
    char ch=in.read();
    if(ch=='\r'||ch=='\n') {
      if(r.n&&!r.overflow){r.buf[r.n]=0;command(r.buf);}r.n=0;r.overflow=false;
    } else if(r.n<sizeof(r.buf)-1&&!r.overflow)r.buf[r.n++]=ch;else r.overflow=true;
  }
}

void pollCommands() {
  static LineReader usb;pollStream(Serial,usb);
#if USE_BT
  static LineReader bt;pollStream(SerialBT,bt);
  bool connected=SerialBT.hasClient();
  if(connected!=bt_was_connected) {
    bt_was_connected=connected;
    if(connected){emit("# bt client connected\n");config();}
    else Serial.print("# bt client disconnected\n");
  }
#endif
}

void startPair() {
  // Snapshot both commanded angles: a P or D received between halves affects only the next pair.
  float local_center=constrain(center,PHI_LO+delta,PHI_HI-delta);
  const float s=plus_first?1.f:-1.f;
  pair_second=local_center-s*delta;           // commanded; moveHorn returns the actual angle
  float first=moveHorn(local_center+s*delta);
  if(plus_first)pair_pp=first;else pair_pm=first;
  measuring_second=false;
}

void setup() {
  // B2+R2 are ~370 bytes per pair: buffer them so a line never blocks the sampling loop.
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  initWindow();
#if USE_BT
  if(!SerialBT.begin(BT_NAME))Serial.print("# WARN bluetooth failed to start; USB only\n");
  else Serial.printf("# bluetooth SPP name %s\n",BT_NAME);
#endif
  analogReadResolution(12);analogSetPinAttenuation(MIC_PIN,ADC_11db);
  if(!ledcAttach(SERVO_PIN,50,16)){emit("# FATAL servo PWM setup\n");failed=true;return;}
  servoWrite(SERVO_CENTER);delay(500);config();startPair();
}

void loop() {
  pollCommands();
  if(failed){delay(10);return;}
  if((int32_t)(millis()-ready_ms)<0){delay(1);return;}
  if(!measuring_second) {
    c1=capture();
    float second=moveHorn(pair_second);
    if(plus_first)pair_pm=second;else pair_pp=second;
    measuring_second=true;return;
  }
  Capture c2=capture();
  const bool pf=plus_first;
  const Capture &cm=pf?c2:c1, &cp=pf?c1:c2;   // A- / A+ regardless of order
  bool clip=cm.clip||cp.clip,late=cm.late||cp.late;
  bool valid=!clip&&!late&&cm.A>=A_MIN&&cp.A>=A_MIN&&cm.A>=SNR_MIN*cm.N&&cp.A>=SNR_MIN*cp.N;
  unsigned flags=(valid?1:0)|(clip?2:0)|(late?4:0)|(pf?8:0);
  static char bins[520];
  int n=snprintf(bins,sizeof(bins),"B2,%lu,%d",(unsigned long)seq,M_TONES);
  for(int m=0;m<M_TONES&&n<(int)sizeof(bins);m++)n+=snprintf(bins+n,sizeof(bins)-n,",%.2f",cm.tone[m]);
  for(int m=0;m<M_TONES&&n<(int)sizeof(bins);m++)n+=snprintf(bins+n,sizeof(bins)-n,",%.2f",cp.tone[m]);
  if(n<(int)sizeof(bins))snprintf(bins+n,sizeof(bins)-n,",%.3f,%.3f",cm.N,cp.N);
  char payload[240];
  snprintf(payload,sizeof(payload),"R2,%lu,%lu,%lu,%lu,%.1f,%.1f,%.4f,%.4f,%.4f,%.4f,%u,%u,%u",
    (unsigned long)seq++,(unsigned long)cm.midpoint,(unsigned long)cp.midpoint,(unsigned long)micros(),
    pair_pm,pair_pp,cm.A,cp.A,cm.N,cp.N,cm.pp,cp.pp,flags);
  // Start the next servo move before printing, so serial output overlaps the slew.
  plus_first=!plus_first;
  startPair();
  emitf("%s*%04X\n",bins,crc16(bins));
  emitf("%s*%04X\n",payload,crc16(payload));
}
