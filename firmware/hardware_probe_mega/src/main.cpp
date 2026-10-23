// Temporary diagnostics: no motion controller or EEPROM writes. Motion is limited to
// "SPIN m +|-" (one motor, PWM 90/255, stops after 0.8 s) for identifying wheels, and
// "MOT a b c d" manual drive (raw signed PWM for M1..M4, |v| <= 200, stops unless
// refreshed within 0.4 s). STOP, '!' or a malformed line stops all motors. If the program
// freezes, a watchdog interrupt at 0.5 s cuts every motor PWM, and a reset follows at 1 s.
// Colour S0/S1 = D28/D25 (20% scaling) and S2/S3 = D30/D29 (filter) are driven since 23 Oct.
// The only writes beyond PWM inhibit go to volatile registers of an identified MPU gyro.
// Gyro: shield IMU header on the Mega TWI bus (SDA D20 / SCL D21), shared with OLED 0x3c.
// Bluetooth: HC-05/06-class SPP module on Serial3 (TX3 D14 / RX3 D15) at 9600 baud.
// It gets every event line, a compact sample at 2 Hz (9600 baud cannot carry the full
// 4 Hz USB stream), and accepts the same no-motion commands as USB. Output is sent only
// within 5 s of a host "PING": an unconnected module parses its input as AT commands.
#include <Arduino.h>
#include <Wire.h>
#include <avr/wdt.h>
#include "../../core_robot_mega/include/frequency_color.h"
#include "../../core_robot_mega/include/mpu_gyro.h"
extern "C" uint8_t colorPulseBurst();
static_assert(F_CPU==core::COLOR_CPU_HZ,"Probe sampling requires a 16 MHz Mega");
static const uint8_t pwm[]={12,8,9,5};
// M1..M4 direction pins, updated-shield profile (M4 on A4/A5), as in config.h.
static const uint8_t motorIn1[]={34,37,43,A4},motorIn2[]={35,36,42,A5};
static const uint8_t SPIN_PWM=90,DRIVE_PWM_MAX=200;
static const uint16_t SPIN_MS=800,DRIVE_WATCHDOG_MS=400;
static bool motorsLive=false;static int8_t spinning=-1;static uint32_t motorDeadline=0;
// Soft start: outputs step toward their targets by RAMP_STEP every RAMP_MS (0->100 in
// ~125 ms), limiting the inrush that reset the board with four motors starting at once.
// Stopping is never ramped.
static const uint8_t RAMP_STEP=8,RAMP_MS=10;
static int16_t motorTarget[4]={},motorNow[4]={};static uint32_t lastRampMs=0;
// The bootloader clears MCUSR, so the watchdog's early interrupt leaves its own mark in
// RAM that a reset does not initialise. Any other value means power-on/brown-out/external.
static const uint16_t WATCHDOG_MARK=0xA55A;
static uint16_t resetMark __attribute__((section(".noinit")));
// Set while running. Retained RAM after a reset without the watchdog mark means a brown-out
// or reset line (with no USB cable that is almost certainly brown-out); lost RAM = power-on.
static const uint16_t RUNNING_MARK=0x5AA5;
static uint16_t runningMark __attribute__((section(".noinit")));
static bool lastResetWatchdog=false;
static const __FlashStringHelper* bootCause=0;
// Left PL1/PL2 (D48/D47), right PG1/PG0 (D40/D41) since the plug swap.
static const uint8_t trig[]={48,40},echo[]={47,41},light[]={A0,A7,A2,A6}; // LDRs left to right; A1/A3 not used, A4/A5 are M4.
// User-reported 2026-10-23: gyro INT wire on D33 (PC4, polled; no interrupt on PORTC).
static const uint8_t gyroInt=33;
static const uint8_t colorS0=28,colorS1=25,colorS2=30,colorS3=29;
static const unsigned long BLUETOOTH_BAUD=9600;
static uint32_t lastSample=0,sampleCount=0,btQuietUntil=0,btPingMs=0;
static bool btPinged=false;
static bool btHostPresent() { return btPinged&&millis()-btPingMs<5000&&int32_t(millis()-btQuietUntil)>=0; }
struct LineReader { char line[32];uint8_t used=0;bool dropping=false; };
static LineReader usbReader,btReader;
// Event lines go to both links.
class BothLinks : public Print {
 public:
  size_t write(uint8_t c) override { Serial.write(c);if(btHostPresent()) Serial3.write(c);return 1; }
};
static BothLinks out;
static int16_t accelRaw[3]={};static bool accelOk=false;

static uint8_t gyroAddress=0,gyroWho=0;
static const char* gyroModel=0;
static bool gyroRunning=false,anglesValid=false,intLast=false;
static float bias[3]={},biasSd[3]={},angle[3]={};
static int16_t lastRaw[3]={};
static uint32_t frames=0,intRises=0,intDrains=0,timerDrains=0,busErrors=0,lastDrainMs=0;
static uint16_t overflows=0,saturated=0;

static void inhibitMotors() {
  // PWM is common to both known shield revisions. Direction/encoder pins untouched.
  for(uint8_t pin:pwm) { digitalWrite(pin,LOW);pinMode(pin,OUTPUT); }
}
ISR(WDT_vect) {
  // Program frozen for 0.5 s: cut motor power now; hardware resets at the next timeout.
  for(uint8_t pin:pwm) digitalWrite(pin,LOW);
  resetMark=WATCHDOG_MARK;
}
static void stopMotors() {
  inhibitMotors();
  if(motorsLive) for(uint8_t m=0;m<4;++m) { digitalWrite(motorIn1[m],LOW);digitalWrite(motorIn2[m],LOW); }
  motorsLive=false;spinning=-1;
  for(uint8_t m=0;m<4;++m) motorTarget[m]=motorNow[m]=0;
}
// Positive = IN1 HIGH / IN2 LOW, matching the core's speed>=0 convention.
static void applyMotor(uint8_t m,int16_t value) {
  pinMode(motorIn1[m],OUTPUT);pinMode(motorIn2[m],OUTPUT);
  analogWrite(pwm[m],0);
  digitalWrite(motorIn1[m],value>0?HIGH:LOW);digitalWrite(motorIn2[m],value<0?HIGH:LOW);
  if(value) analogWrite(pwm[m],min(abs(value),int16_t(DRIVE_PWM_MAX)));
}
static void setMotor(uint8_t m,int16_t value) {
  motorTarget[m]=constrain(value,-int16_t(DRIVE_PWM_MAX),int16_t(DRIVE_PWM_MAX));
}
static void rampMotors() {
  if(!motorsLive||millis()-lastRampMs<RAMP_MS) return;
  lastRampMs=millis();
  for(uint8_t m=0;m<4;++m) {
    int16_t now=motorNow[m],target=motorTarget[m];
    if(now==target) continue;
    // Linear steps pass through zero on reversal, so direction flips at zero output.
    now+=constrain(target-now,-int16_t(RAMP_STEP),int16_t(RAMP_STEP));
    motorNow[m]=now;applyMotor(m,now);
  }
}
static bool parseDrive(const char* text,int16_t* v) {
  for(uint8_t m=0;m<4;++m) {
    char* end;long value=strtol(text,&end,10);
    if(end==text||value<-255||value>255) return false;
    v[m]=value;text=end;
  }
  while(*text==' ') ++text;
  return !*text;
}
static bool reg(uint8_t address,uint8_t index,uint8_t* data,uint8_t count) {
  Wire.beginTransmission(address);Wire.write(index);
  if(Wire.endTransmission(false)!=0||Wire.requestFrom(address,count)!=count) return false;
  for(uint8_t i=0;i<count;++i) data[i]=Wire.read();
  return true;
}
static void scan() {
  out.print(F("{\"type\":\"i2c\",\"bus\":\"D20/D21\",\"addresses\":["));bool first=true;
  for(uint8_t address=8;address<120;++address) {
    Wire.beginTransmission(address);
    if(Wire.endTransmission()==0) { if(!first) out.print(',');out.print(address);first=false; }
  }
  out.println(F("],\"note\":\"ACK does not identify a device model\"}"));
}
static int16_t range(uint8_t channel) {
  digitalWrite(trig[channel],LOW);delayMicroseconds(2);
  digitalWrite(trig[channel],HIGH);delayMicroseconds(10);digitalWrite(trig[channel],LOW);
  uint32_t duration=pulseIn(echo[channel],HIGH,24000UL);
  int16_t mm=duration?duration*0.1715f:-1;
  return mm>=20&&mm<=4000?mm:-1;
}

static bool gyroWrite(uint8_t index,uint8_t value) {
  Wire.beginTransmission(gyroAddress);Wire.write(index);Wire.write(value);
  return Wire.endTransmission()==0;
}
static void fifoRestart() {
  gyroWrite(core::MPU_USER_CTRL,core::MPU_USER_FIFO_RESET);
  gyroWrite(core::MPU_USER_CTRL,core::MPU_USER_FIFO_ON);
}
// Drain whole XYZ frames. Status is read first: any register read clears it.
static uint16_t gyroDrain(float* sum,float* squares) {
  uint8_t status,count[2],data[30]; // Wire buffers 32 bytes: five XYZ frames.
  if(!reg(gyroAddress,core::MPU_INT_STATUS,&status,1)||
     !reg(gyroAddress,core::MPU_FIFO_COUNT,count,2)) { ++busErrors;return 0; }
  uint16_t bytes=(uint16_t(count[0])<<8)|count[1];
  if((status&core::MPU_INT_FIFO_OVERFLOW)||bytes>=core::MPU_FIFO_BYTES_MIN) {
    ++overflows;anglesValid=false;fifoRestart();return 0;
  }
  uint16_t drained=0;bytes-=bytes%6;
  while(bytes) {
    uint8_t chunk=bytes>sizeof(data)?sizeof(data):bytes;
    if(!reg(gyroAddress,core::MPU_FIFO_RW,data,chunk)) {
      ++busErrors;anglesValid=false;fifoRestart();return drained;
    }
    for(uint8_t at=0;at<chunk;at+=6,++drained) for(uint8_t axis=0;axis<3;++axis) {
      int16_t raw=core::mpuWord(data+at+axis*2);lastRaw[axis]=raw;
      if(core::mpuSaturated(raw)) { ++saturated;anglesValid=false; }
      angle[axis]+=core::mpuSampleDegrees(raw,bias[axis]);
      if(sum) { sum[axis]+=raw;squares[axis]+=float(raw)*raw; }
    }
    bytes-=chunk;
  }
  frames+=drained;return drained;
}
static void gyroService() {
  wdt_reset();
  if(!gyroRunning) return;
  bool level=digitalRead(gyroInt);if(level&&!intLast) ++intRises;intLast=level;
  uint32_t now=millis();
  // Data-ready latches INT high; the timer path keeps draining if INT is miswired.
  if(level&&now-lastDrainMs>=20) ++intDrains;
  else if(now-lastDrainMs>=60) ++timerDrains;
  else return;
  lastDrainMs=now;gyroDrain(0,0);
}
// Robot must be stationary. Two seconds of frames give bias and noise per axis.
static void gyroZero() {
  if(!gyroRunning) return;
  float sum[3]={},squares[3]={};uint16_t total=0;fifoRestart();
  uint32_t begin=millis();uint16_t before=overflows;
  while(total<200&&millis()-begin<3000) { wdt_reset();delay(20);total+=gyroDrain(sum,squares); }
  for(uint8_t axis=0;axis<3;++axis) {
    float mean=total?sum[axis]/total:0;
    bias[axis]=mean;biasSd[axis]=total?sqrt(max(0.0f,squares[axis]/total-mean*mean)):0;
    angle[axis]=0;
  }
  anglesValid=total>=200&&overflows==before;
  out.print(F("{\"type\":\"gyro_zero\",\"frames\":"));out.print(total);
  out.print(F(",\"bias_raw\":["));
  for(uint8_t axis=0;axis<3;++axis) { if(axis) out.print(',');out.print(bias[axis],2); }
  out.print(F("],\"noise_sd_raw\":["));
  for(uint8_t axis=0;axis<3;++axis) { if(axis) out.print(',');out.print(biasSd[axis],2); }
  out.print(F("],\"valid\":"));out.print(anglesValid);
  out.println(F(",\"note\":\"valid only if the robot was stationary\"}"));
}
static void gyroBegin() {
  gyroRunning=anglesValid=false;gyroAddress=gyroWho=0;
  pinMode(gyroInt,INPUT);
  for(uint8_t address=0x68;address<=0x69&&!gyroAddress;++address)
    if(reg(address,core::MPU_WHO_AM_I,&gyroWho,1)) gyroAddress=address;
  gyroModel=gyroAddress?core::mpuModel(gyroWho):0;
  if(!gyroModel) return;
  bool ok=gyroWrite(core::MPU_PWR_MGMT_1,0x80);delay(100); // Volatile device reset.
  for(const core::MpuWrite& w:core::MPU_SETUP) ok=gyroWrite(w.reg,w.value)&&ok;
  ok=gyroWrite(core::MPU_FIFO_EN,core::MPU_FIFO_GYRO_BIT[0]|core::MPU_FIFO_GYRO_BIT[1]|
    core::MPU_FIFO_GYRO_BIT[2])&&ok;
  delay(200); // Gyro start-up and DLPF settling before the bias window.
  gyroRunning=ok;gyroZero();lastDrainMs=millis();
}
static void gyroReport() {
  Serial.print(F("{\"bus\":\"D20/D21\",\"address\":"));if(gyroAddress) Serial.print(gyroAddress);else Serial.print(F("null"));
  Serial.print(F(",\"whoami\":"));if(gyroAddress) Serial.print(gyroWho);else Serial.print(F("null"));
  Serial.print(F(",\"model\":"));
  if(gyroModel) { Serial.print('"');Serial.print(gyroModel);Serial.print('"'); } else Serial.print(F("null"));
  Serial.print(F(",\"running\":"));Serial.print(gyroRunning);
  Serial.print(F(",\"int_pin\":"));Serial.print(digitalRead(gyroInt));
  if(gyroRunning) {
    Serial.print(F(",\"gyro_raw\":["));
    for(uint8_t axis=0;axis<3;++axis) { if(axis) Serial.print(',');Serial.print(lastRaw[axis]); }
    Serial.print(F("],\"rate_dps\":["));
    for(uint8_t axis=0;axis<3;++axis) {
      if(axis) Serial.print(',');
      Serial.print((lastRaw[axis]-bias[axis])/core::MPU_GYRO_LSB_PER_DPS,2);
    }
    Serial.print(F("],\"angle_deg\":["));
    for(uint8_t axis=0;axis<3;++axis) { if(axis) Serial.print(',');Serial.print(angle[axis],2); }
    Serial.print(F("],\"angles_valid\":"));Serial.print(anglesValid);
    // Gravity shows which axis is vertical, i.e. which gyro axis measures yaw.
    Serial.print(F(",\"accel_raw\":"));
    if(accelOk) {
      Serial.print('[');
      for(uint8_t axis=0;axis<3;++axis) { if(axis) Serial.print(',');Serial.print(accelRaw[axis]); }
      Serial.print(']');
    } else Serial.print(F("null"));
    Serial.print(F(",\"frames\":"));Serial.print(frames);
    Serial.print(F(",\"int_rises\":"));Serial.print(intRises);
    Serial.print(F(",\"int_drains\":"));Serial.print(intDrains);
    Serial.print(F(",\"timer_drains\":"));Serial.print(timerDrains);
    Serial.print(F(",\"overflows\":"));Serial.print(overflows);
    Serial.print(F(",\"saturated\":"));Serial.print(saturated);
  }
  Serial.print(F(",\"bus_errors\":"));Serial.print(busErrors);
  Serial.print('}');
}
static void imu(uint8_t address) {
  uint8_t who=0,power=0,bytes[2]={};
  bool ok=reg(address,0x75,&who,1);
  Serial.print(F("{\"address\":"));Serial.print(address);
  Serial.print(F(",\"who_read\":"));Serial.print(ok?1:0);
  Serial.print(F(",\"whoami\":"));if(ok) Serial.print(who);else Serial.print(F("null"));
  Serial.print(F(",\"power\":"));if(ok&&reg(address,0x6b,&power,1)) Serial.print(power);else Serial.print(F("null"));
  Serial.print(F(",\"gyro_z_raw\":"));
  if(ok&&reg(address,0x47,bytes,2)) Serial.print(int16_t((uint16_t(bytes[0])<<8)|bytes[1]));else Serial.print(F("null"));
  Serial.print('}');
}
static void readAccel() {
  uint8_t accel[6];
  accelOk=gyroRunning&&reg(gyroAddress,core::MPU_ACCEL_XOUT,accel,6);
  if(gyroRunning&&!accelOk) ++busErrors;
  for(uint8_t axis=0;axis<3&&accelOk;++axis) accelRaw[axis]=core::mpuWord(accel+axis*2);
}
template<typename T> static void list(Print& o,const T* v,uint8_t n,uint8_t digits=0) {
  o.print('[');
  for(uint8_t i=0;i<n;++i) { if(i) o.print(',');if(digits) o.print(float(v[i]),digits);else o.print(v[i]); }
  o.print(']');
}
// Same keys as the USB sample, minus diagnostics-only fields: about 250 bytes.
static void compactSample(Print& o,const uint16_t* adc,int16_t left,int16_t right,const uint16_t* rgbc) {
  float rate[3];
  for(uint8_t axis=0;axis<3;++axis) rate[axis]=(lastRaw[axis]-bias[axis])/core::MPU_GYRO_LSB_PER_DPS;
  o.print(F("{\"type\":\"sample\",\"link\":\"bt\",\"ms\":"));o.print(millis());
  o.print(F(",\"boot\":\""));o.print(bootCause);o.print('"');
  o.print(F(",\"adc\":"));list(o,adc,4);
  o.print(F(",\"ultrasonic_mm\":["));o.print(left);o.print(',');o.print(right);o.print(']');
  o.print(F(",\"gyro\":{\"model\":"));
  if(gyroModel) { o.print('"');o.print(gyroModel);o.print('"'); } else o.print(F("null"));
  if(gyroRunning) {
    o.print(F(",\"angle_deg\":"));list(o,angle,3,1);
    o.print(F(",\"rate_dps\":"));list(o,rate,3,1);
    o.print(F(",\"angles_valid\":"));o.print(anglesValid);
    if(accelOk) { o.print(F(",\"accel_raw\":"));list(o,accelRaw,3); }
    o.print(F(",\"int_rises\":"));o.print(intRises);
  }
  o.print(F(",\"bus_errors\":"));o.print(busErrors);
  o.print(F("},\"color_rgbc\":"));list(o,rgbc,4);o.println('}');
}
static void service(); // Commands, motor watchdog and gyro, also called mid-sample.
static bool sampling=false;
static void sample() {
  if(sampling) return;
  sampling=true;
  if(!motorsLive) inhibitMotors();
  uint16_t value[4]={};
  for(uint8_t i=0;i<4;++i) { analogRead(light[i]);for(uint8_t n=0;n<16;++n) value[i]+=analogRead(light[i]);value[i]=(value[i]+8)/16; }
  int16_t left=range(0);
  for(uint32_t t=millis();millis()-t<40;) service(); // Ping spacing, still serviced.
  int16_t right=range(1);service();
  // Channels R, G, B, clear; 3 ms filter settling, then 96 bursts (6.912 ms exposure).
  uint16_t rgbc[4]={};
  for(uint8_t channel=0;channel<4;++channel) {
    digitalWrite(colorS2,core::frequencyS2(channel)?HIGH:LOW);
    digitalWrite(colorS3,core::frequencyS3(channel)?HIGH:LOW);
    delay(3);
    for(uint16_t burst=0;burst<core::COLOR_MIN_BURSTS;++burst) {
      uint8_t status=SREG;cli();uint8_t count=colorPulseBurst();SREG=status;
      rgbc[channel]+=count;delayMicroseconds((burst*13U)&31U);
    }
    service();
  }
  Serial.print(F("{\"type\":\"sample\",\"ms\":"));Serial.print(millis());
  Serial.print(F(",\"boot\":\""));Serial.print(bootCause);Serial.print('"');
  Serial.print(F(",\"motors\":\"PWM_LOW\",\"light_pins\":[\"A0\",\"A7\",\"A2\",\"A6\"],\"adc\":["));
  for(uint8_t i=0;i<4;++i) { if(i) Serial.print(',');Serial.print(value[i]); }
  Serial.print(F("],\"ultrasonic_mm\":["));Serial.print(left);Serial.print(',');Serial.print(right);
  Serial.print(F("],\"imu\":["));imu(0x68);Serial.print(',');imu(0x69);
  Serial.print(F("],\"gyro\":"));gyroService();readAccel();gyroReport(); // No output mid-line.
  Serial.print(F(",\"color_out_pin\":32,\"color_rgbc\":["));
  for(uint8_t i=0;i<4;++i) { if(i) Serial.print(',');Serial.print(rgbc[i]); }
  Serial.print(']');
  Serial.print(F(",\"color_exposure_cycles\":"));Serial.print(uint32_t(core::COLOR_MIN_BURSTS)*core::COLOR_BURST_CYCLES);
  Serial.println(F(",\"color_filters\":\"DRIVEN_20PCT\"}"));
  if(++sampleCount%2==0&&btHostPresent()) compactSample(Serial3,value,left,right,rgbc);
  sampling=false;
}
// USB only: send AT text to the Bluetooth module (works only while no device is
// connected over Bluetooth) and report its raw reply. "BTAT x" sends x bare,
// "BTATN x" appends CRLF; module families differ in which they expect.
static void btAt(const char* text,bool crlf) {
  // An unconnected module parses everything it receives as AT input, so samples
  // must not interleave with configuration: pause them for 15 s after each AT.
  btQuietUntil=millis()+15000;Serial3.flush();delay(300);
  while(Serial3.available()) Serial3.read();
  Serial3.print(text);if(crlf) Serial3.print(F("\r\n"));
  char reply[64];uint8_t n=0;uint32_t start=millis();
  while(millis()-start<3000) {
    gyroService();
    while(Serial3.available()) { char c=Serial3.read();if(n<sizeof(reply)-1&&c>=32&&c<127&&c!='"'&&c!='\\') reply[n++]=c;else if(n<sizeof(reply)-1&&(c=='\r'||c=='\n')) reply[n++]='|'; }
  }
  reply[n]=0;btQuietUntil=millis()+15000;
  Serial.print(F("{\"type\":\"bt_at\",\"sent\":\""));Serial.print(text);
  Serial.print(F("\",\"crlf\":"));Serial.print(crlf);
  Serial.print(F(",\"reply\":\""));Serial.print(reply);Serial.println(F("\"}"));
}
static void command(const char* line,bool usb) {
  if(!strcmp(line,"PING")) { if(!usb) { btPinged=true;btPingMs=millis(); } return; }
  if(usb&&!strcmp(line,"HANGTEST")) { // Proves the watchdog: freeze; expect a fresh "ready".
    stopMotors();Serial.println(F("{\"type\":\"hangtest\",\"expect\":\"reset within ~1 s\"}"));Serial.flush();
    for(;;) {}
  }
  if(usb&&!strncmp(line,"BTATN ",6)) { btAt(line+6,true);return; }
  if(usb&&!strncmp(line,"BTAT ",5)) { btAt(line+5,false);return; }
  if(!strncmp(line,"SPIN ",5)&&line[5]>='1'&&line[5]<='4'&&line[6]==' '&&(line[7]=='+'||line[7]=='-')&&!line[8]) {
    stopMotors();
    uint8_t m=line[5]-'1';bool positive=line[7]=='+';
    setMotor(m,positive?SPIN_PWM:-int16_t(SPIN_PWM));
    motorsLive=true;spinning=m;motorDeadline=millis()+SPIN_MS;
    out.print(F("{\"type\":\"spin\",\"motor\":"));out.print(m+1);
    out.print(F(",\"dir\":\""));out.print(line[7]);out.print(F("\",\"pwm\":"));out.print(SPIN_PWM);
    out.print(F(",\"ms\":"));out.print(SPIN_MS);out.println('}');
    return;
  }
  if(!strncmp(line,"MOT ",4)) {
    int16_t v[4];
    if(!parseDrive(line+4,v)) { stopMotors();out.println(F("{\"error\":\"BAD_MOT_STOPPED\"}"));return; }
    if(spinning>=0) stopMotors();
    for(uint8_t m=0;m<4;++m) setMotor(m,v[m]);
    motorsLive=true;motorDeadline=millis()+DRIVE_WATCHDOG_MS;
    return;
  }
  if(!strcmp(line,"STOP")) { stopMotors();out.println(F("{\"type\":\"stop\",\"motors\":\"PWM_LOW\"}")); }
  else if(!strcmp(line,"PROBE")) scan();
  else if(!strcmp(line,"STATUS")) sample();
  else if(!strcmp(line,"GYROZERO")) gyroZero();
  else if(!strcmp(line,"GYROINIT")) { gyroBegin();scan(); }
  else if(!strcmp(line,"HEADING0")) {
    for(float& a:angle) a=0;
    anglesValid=gyroRunning;
    out.println(F("{\"type\":\"heading0\"}"));
  }
  else if(!strcmp(line,"HELP")) out.println(F("{\"commands\":[\"STOP\",\"PROBE\",\"STATUS\",\"GYROINIT\",\"GYROZERO\",\"HEADING0\",\"PING\",\"SPIN m +|-\",\"MOT a b c d\",\"HELP\"],\"motion\":\"SPIN/MOT, watchdogged\"}"));
  else out.println(F("{\"error\":\"UNKNOWN_COMMAND_NO_MOTION\"}"));
}
static void commands(Stream& in,LineReader& r,bool usb) {
  while(in.available()) {
    char c=in.read();
    if(c=='!') { stopMotors();r.used=0;r.dropping=true;continue; }
    if(c=='\r') continue;
    if(c=='\n') {
      if(r.dropping) { r.dropping=false;r.used=0;continue; }
      r.line[r.used]=0;r.used=0;command(r.line,usb);
    } else if(r.used<sizeof(r.line)-1) r.line[r.used++]=c;
    else { r.used=0;r.dropping=true;stopMotors(); }
  }
}
void setup() {
  // Clear a watchdog reset flag first, or the next timeout could loop the bootloader.
  MCUSR=0;wdt_disable();
  lastResetWatchdog=resetMark==WATCHDOG_MARK;
  bootCause=lastResetWatchdog?F("watchdog"):runningMark==RUNNING_MARK?F("brownout_or_reset_line"):F("power_on");
  resetMark=0;runningMark=RUNNING_MARK;
  inhibitMotors();Serial.begin(115200);Serial3.begin(BLUETOOTH_BAUD);Wire.begin();Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(2500,true);
#endif
  for(uint8_t i=0;i<2;++i) { digitalWrite(trig[i],LOW);pinMode(trig[i],OUTPUT);pinMode(echo[i],INPUT); }
  // A4 and every motor direction pin remain high impedance.
  for(uint8_t pin:light) pinMode(pin,INPUT);
  pinMode(32,INPUT);
  digitalWrite(colorS0,HIGH);digitalWrite(colorS1,LOW);
  pinMode(colorS0,OUTPUT);pinMode(colorS1,OUTPUT);pinMode(colorS2,OUTPUT);pinMode(colorS3,OUTPUT);
  delay(250);
  // Interrupt-then-reset mode, 0.5 s: the ISR stops the motors, the next timeout resets.
  cli();wdt_reset();
  WDTCSR=_BV(WDCE)|_BV(WDE);
  WDTCSR=_BV(WDIE)|_BV(WDE)|_BV(WDP2)|_BV(WDP0);
  sei();
  out.print(F("{\"type\":\"ready\",\"firmware\":\"HARDWARE_PROBE_2\",\"motion\":\"SPIN/MOT, watchdogged\",\"eeprom_writes\":false"));
  out.print(F(",\"reset_cause\":\""));
  out.print(bootCause);
  out.println(F("\"}"));
  gyroBegin();
  scan();
}
static void service() {
  commands(Serial,usbReader,true);
  commands(Serial3,btReader,false);
  gyroService();
  rampMotors();
  if(motorsLive&&int32_t(millis()-motorDeadline)>=0) {
    int8_t m=spinning;stopMotors();
    if(m>=0) { out.print(F("{\"type\":\"spin_done\",\"motor\":"));out.print(m+1);out.println('}'); }
    else out.println(F("{\"type\":\"drive_timeout_stop\"}"));
  }
}
void loop() {
  service();
  if(millis()-lastSample>=250) { lastSample=millis();sample(); }
}
