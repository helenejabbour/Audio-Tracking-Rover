/* Acoustic frontend V2. Classic ESP32, Arduino-ESP32 3.x, USB 115200.
 * No external servo library. GPIO34=ADC1 input, GPIO32=horn PWM.
 * Continuous 4 kHz beacon; paired captures have separate midpoint timestamps.
 * R2,seq,tm_us,tp_us,emit_us,phi_m,phi_p,Am,Ap,Nm,Np,pp_m,pp_p,flags*CRC16
 * CRC16-CCITT-FALSE: ASCII before '*', init FFFF, polynomial 1021.
 * flags: 1 valid, 2 ADC clipping, 4 sample timing failure; valid excludes 2/4.
 * Commands P<deg>, D<deg>, S<ms>, ?; one newline-terminated command per line.
 * Servo calibration constants MUST be verified with the horn disconnected first.
 */
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

constexpr int MIC_PIN=34, SERVO_PIN=32;
constexpr int FS=16000, NS=512, F0=4000, FREF=3000;
constexpr int SERVO_CENTER=90, SERVO_SIGN=1, SERVO_MIN=15, SERVO_MAX=165;
constexpr float HORN_LIMIT=65.f, SLEW_DEG_S=300.f;
constexpr int PULSE_MIN_US=600, PULSE_MAX_US=2400;
constexpr float A_MIN=3.f, SNR_MIN=4.f;
static_assert(F0*NS%FS==0 && FREF*NS%FS==0, "Goertzel bins must be integral");
static_assert(SERVO_CENTER>=SERVO_MIN && SERVO_CENTER<=SERVO_MAX, "invalid center");
float center=0.f, delta=20.f;
int settle_ms=40, servo_angle=SERVO_CENTER;
uint32_t seq=0, ready_ms=0;
float pair_pm=0.f, pair_pp=0.f;
bool measuring_plus=false, failed=false;
struct Capture { float A,N; uint16_t pp; uint32_t midpoint; bool clip,late; } cm;

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
  phi=constrain(phi,-HORN_LIMIT,HORN_LIMIT);
  int angle=constrain(SERVO_CENTER+SERVO_SIGN*(int)lroundf(phi),SERVO_MIN,SERVO_MAX);
  int travel=abs(angle-servo_angle);
  servoWrite(angle); servo_angle=angle;
  ready_ms=millis()+(uint32_t)ceilf(1000.f*travel/SLEW_DEG_S)+settle_ms;
  return (float)(SERVO_SIGN*(angle-SERVO_CENTER));
}

float binAmplitude(const int16_t *x,float mean,int k) {
  float coeff=2*cosf(2*PI*k/NS),s1=0,s2=0;
  for(int i=0;i<NS;i++){float s=(x[i]-mean)+coeff*s1-s2;s2=s1;s1=s;}
  return 2*sqrtf(fmaxf(0,s1*s1+s2*s2-coeff*s1*s2))/NS;
}

Capture capture() {
  int16_t x[NS]; long sum=0; int mn=4095,mx=0,late=0;
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
  c.A=binAmplitude(x,(float)sum/NS,F0*NS/FS);
  c.N=binAmplitude(x,(float)sum/NS,FREF*NS/FS);
  c.pp=mx-mn;c.midpoint=first+(uint32_t)(last-first)/2;
  c.clip=mn<=50||mx>=4045;
  c.late=late>NS/20||fabsf(actual_fs/FS-1)>0.02f;
  return c;
}

void config() {
  Serial.printf("C2,fs=%d,N=%d,tone=%d,reference=%d,settle_ms=%d,center=%d,sign=%d,limits=%d:%d\n",
    FS,NS,F0,FREF,settle_ms,SERVO_CENTER,SERVO_SIGN,SERVO_MIN,SERVO_MAX);
}

void command(const char *s) {
  if(s[0]=='?' && s[1]==0){config();return;}
  char *end;float value=strtof(s+1,&end);
  if(end==s+1||*end!=0||!isfinite(value)){Serial.println("# invalid command");return;}
  switch(s[0]) {
    case 'P':center=constrain(value,-HORN_LIMIT+delta,HORN_LIMIT-delta);break;
    case 'D':delta=constrain(value,2.f,40.f);center=constrain(center,-HORN_LIMIT+delta,HORN_LIMIT-delta);break;
    case 'S':settle_ms=constrain((int)value,20,1000);break;
    default:Serial.println("# unknown command");break;
  }
}

void pollCommands() {
  static char buf[32];static uint8_t n=0;static bool overflow=false;
  while(Serial.available()) {
    char ch=Serial.read();
    if(ch=='\r'||ch=='\n') {
      if(n&&!overflow){buf[n]=0;command(buf);}n=0;overflow=false;
    } else if(n<sizeof(buf)-1&&!overflow)buf[n++]=ch;else overflow=true;
  }
}

void startPair() {
  // Snapshot both commanded angles: a P or D received between halves affects only the next pair.
  float local_center=constrain(center,-HORN_LIMIT+delta,HORN_LIMIT-delta);
  pair_pp=local_center+delta;
  pair_pm=moveHorn(local_center-delta);
  measuring_plus=false;
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);analogSetPinAttenuation(MIC_PIN,ADC_11db);
  if(!ledcAttach(SERVO_PIN,50,16)){Serial.println("# FATAL servo PWM setup");failed=true;return;}
  servoWrite(SERVO_CENTER);delay(500);config();startPair();
}

void loop() {
  pollCommands();
  if(failed){delay(10);return;}
  if((int32_t)(millis()-ready_ms)<0){delay(1);return;}
  if(!measuring_plus) {cm=capture();pair_pp=moveHorn(pair_pp);measuring_plus=true;return;}
  Capture cp=capture();
  bool clip=cm.clip||cp.clip,late=cm.late||cp.late;
  bool valid=!clip&&!late&&cm.A>=A_MIN&&cp.A>=A_MIN&&cm.A>=SNR_MIN*cm.N&&cp.A>=SNR_MIN*cp.N;
  unsigned flags=(valid?1:0)|(clip?2:0)|(late?4:0);
  char payload[240];
  snprintf(payload,sizeof(payload),"R2,%lu,%lu,%lu,%lu,%.1f,%.1f,%.4f,%.4f,%.4f,%.4f,%u,%u,%u",
    (unsigned long)seq++,(unsigned long)cm.midpoint,(unsigned long)cp.midpoint,(unsigned long)micros(),
    pair_pm,pair_pp,cm.A,cp.A,cm.N,cp.N,cm.pp,cp.pp,flags);
  Serial.printf("%s*%04X\n",payload,crc16(payload));
  startPair();
}
