// Temporary diagnostics: no motion controller or EEPROM writes. Motion is limited to
// "SPIN m +|-" (one motor, PWM 90/255, stops after 0.8 s) for identifying wheels, and
// "MOT a b c d" manual drive (raw signed PWM for M1..M4, |v| <= 200, stops unless
// refreshed within 0.4 s), and "TURNTEST a b c d" (see ttEnd). STOP, '!' or a malformed line stops all motors. If the program
// freezes, a watchdog interrupt at 0.5 s cuts every motor PWM, and a reset follows at 1 s.
// Colour S0/S1 = D28/D25 (20% scaling) and S2/S3 = D30/D29 (filter) are driven since 23 Oct.
// The only writes beyond PWM inhibit go to volatile registers of an identified MPU gyro.
// Gyro: shield IMU header on the Mega TWI bus (SDA D20 / SCL D21), shared with OLED 0x3c.
// Bluetooth: HC-05/06-class SPP module on Serial3 (TX3 D14 / RX3 D15) at 9600 baud.
// It gets every event line, a compact sample at 1 Hz and a short F line at the other 5 Hz
// sample times (9600 baud cannot carry the full USB stream), and accepts the same commands. Output is sent only
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
// Ultrasonics, index order kept as they were added: front-left +45 deg PL1/PL2 (D48/D47) and
// front-right -45 deg PG1/PG0 (D40/D41) since the plug swap; front PA0/PA2 (D22/D24), left
// side +90 deg PB4/PB5 (D10/D11) and right side -90 deg PH3/PH4 (D6/D7), wired 27 Oct.
// Used as plain GPIO; the motor PWM channels on the same timers are unaffected.
static const uint8_t US_COUNT=5,trig[US_COUNT]={48,40,22,10,6},echo[US_COUNT]={47,41,24,11,7};
// Consecutive pings face 90-135 deg apart, so a 15 ms gap avoids cross-talk and all five
// fit the 200 ms sample: front-left, right side, front, left side, front-right.
static const uint8_t PING_ORDER[US_COUNT]={0,4,2,3,1};
static const uint8_t light[]={A0,A7,A2,A6}; // LDRs; physically left to right A0,A2,A7,A6 (page reorders). Higher ADC = brighter. A1/A3 not used, A4/A5 are M4.
// User-reported 2026-10-23: gyro INT wire on D33 (PC4, polled; no interrupt on PORTC).
static const uint8_t gyroInt=33;
static const uint8_t colorS0=28,colorS1=25,colorS2=30,colorS3=29;
static const unsigned long BLUETOOTH_BAUD=9600;
static uint32_t lastSample=0,sampleCount=0,btQuietUntil=0,btPingMs=0;
static bool btPinged=false;
static bool btHostPresent() { return btPinged&&millis()-btPingMs<5000&&int32_t(millis()-btQuietUntil)>=0; }
struct LineReader { char line[48];uint8_t used=0;bool dropping=false; };  // Room for REF/WMAP lines.
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
static uint32_t frames=0,intRises=0,intDrains=0,timerDrains=0,busErrors=0,lastDrainMs=0,lastFrameMs=0;
static uint32_t i2cTimeouts=0,gyroRestarts=0,gyroResets=0;static bool gyroNeedsSetup=false;
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
static void haltMotors() {
  inhibitMotors();
  if(motorsLive) for(uint8_t m=0;m<4;++m) { digitalWrite(motorIn1[m],LOW);digitalWrite(motorIn2[m],LOW); }
  motorsLive=false;spinning=-1;
  for(uint8_t m=0;m<4;++m) motorTarget[m]=motorNow[m]=0;
}
// "TURNTEST a b c d": 4 x (turn 180, turn back), run on board and ended by gyro Z, so it
// keeps going with no host link: a link blackout that leaves the test and the uptime
// running means the Mega stayed on. a..d = raw PWM turning the robot one way (from the
// page's wheel map); odd legs run the negation. Any stop cancels it.
static const uint8_t TT_LEGS=8,TT_IDLE=0,TT_RUNNING=1,TT_DONE=2,TT_ABORTED=3;
static const float TT_DEG=180,TT_LEAD_DEG=12; // Power is cut early; braking finishes the turn.
static const uint16_t TT_LEG_TIMEOUT_MS=8000,TT_PAUSE_MS=1000;
static int16_t ttPwm[4]={};
static int8_t ttLeg=-1;static uint8_t ttState=TT_IDLE,ttLegsDone=0;
static bool ttTurning=false;static float ttStartDeg=0;static uint32_t ttSinceMs=0;
static void ttEnd(uint8_t state,const __FlashStringHelper* reason) {
  haltMotors();ttLeg=-1;ttTurning=false;ttState=state;
  out.print(F("{\"type\":\"turntest_end\",\"result\":\""));out.print(state==TT_DONE?F("done"):F("aborted"));
  out.print(F("\",\"reason\":\""));out.print(reason);
  out.print(F("\",\"legs_done\":"));out.print(ttLegsDone);out.print(F(",\"of\":"));out.print(TT_LEGS);
  out.print(F(",\"ms\":"));out.print(millis());out.println('}');
}
// Tournament step tests: the controller is further below, next to the sampler it reads.
static const uint8_t STEP_IDLE=0,STEP_GO=1,STEP_TURN=2;
static uint8_t stepKind=STEP_IDLE;
static void stepEnd(bool reached,const __FlashStringHelper* reason);
static void stepReport(Print& o);
static void stopMotors() {
  if(ttLeg>=0) ttEnd(TT_ABORTED,F("stopped"));
  if(stepKind!=STEP_IDLE) stepEnd(false,F("stopped"));
  haltMotors();
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
static void ttStartLeg() {
  int8_t dir=ttLeg%2?-1:1;
  for(uint8_t m=0;m<4;++m) setMotor(m,dir*ttPwm[m]);
  motorsLive=true;spinning=-1;ttTurning=true;ttStartDeg=angle[2];
  ttSinceMs=millis();motorDeadline=ttSinceMs+DRIVE_WATCHDOG_MS;
}
static void ttService() {
  if(ttLeg<0) return;
  uint32_t now=millis();float turned=fabs(angle[2]-ttStartDeg);
  if(ttTurning) {
    motorDeadline=now+DRIVE_WATCHDOG_MS;
    if(turned>=TT_DEG-TT_LEAD_DEG) { haltMotors();ttTurning=false;ttSinceMs=now; }
    else if(now-ttSinceMs>=TT_LEG_TIMEOUT_MS) ttEnd(TT_ABORTED,F("leg_timeout_no_gyro_progress"));
    return;
  }
  if(now-ttSinceMs<TT_PAUSE_MS) return;
  // Settled: the reported angle includes braking overshoot.
  ttLegsDone=ttLeg+1;
  out.print(F("{\"type\":\"turntest_leg\",\"leg\":"));out.print(ttLegsDone);out.print(F(",\"of\":"));out.print(TT_LEGS);
  out.print(F(",\"turned_deg\":"));out.print(turned,1);out.print(F(",\"angles_valid\":"));out.print(anglesValid);
  out.print(F(",\"ms\":"));out.print(now);out.println('}');
  if(++ttLeg>=TT_LEGS) ttEnd(TT_DONE,F("complete"));else ttStartLeg();
}
static void ttReport(Print& o) {
  static const char* const names[]={"idle","running","done","aborted"};
  o.print(F(",\"turntest\":{\"state\":\""));o.print(names[ttState]);
  o.print(F("\",\"legs_done\":"));o.print(ttLegsDone);o.print(F(",\"of\":"));o.print(TT_LEGS);o.print('}');
}
// Wire here (framework-arduino-avr 5.2.0) has no timeout unless asked, and it does not define
// WIRE_HAS_TIMEOUT, so the former #if-guarded call compiled to nothing. Motor noise on
// SDA/SCL then left a transfer waiting forever and the watchdog rebooted the board.
// The limit covers a whole transfer: a 30-byte FIFO read alone takes ~2.8 ms at 100 kHz.
static const uint32_t I2C_TIMEOUT_US=25000;
static void i2cBegin() { Wire.begin();Wire.setClock(100000);Wire.setWireTimeout(I2C_TIMEOUT_US,true); }
// Free a bus a slave holds mid-byte: up to 9 SCL clocks until SDA is released, then STOP.
static void i2cRecover() {
  Wire.end();pinMode(SDA,INPUT_PULLUP);pinMode(SCL,INPUT_PULLUP);  // Open drain by hand.
  for(uint8_t i=0;i<9&&!digitalRead(SDA);++i) {
    digitalWrite(SCL,LOW);pinMode(SCL,OUTPUT);delayMicroseconds(5);
    pinMode(SCL,INPUT_PULLUP);delayMicroseconds(5);
  }
  digitalWrite(SDA,LOW);pinMode(SDA,OUTPUT);delayMicroseconds(5);  // SDA rises with SCL high.
  pinMode(SDA,INPUT_PULLUP);delayMicroseconds(5);
  i2cBegin();
}
// Call after any failed transfer: only a timeout means the bus itself was stuck.
static bool i2cFailed() {
  if(Wire.getWireTimeoutFlag()) { Wire.clearWireTimeoutFlag();++i2cTimeouts;i2cRecover();gyroNeedsSetup=gyroRunning; }
  return false;
}
static bool reg(uint8_t address,uint8_t index,uint8_t* data,uint8_t count) {
  Wire.beginTransmission(address);Wire.write(index);
  if(Wire.endTransmission(false)!=0||Wire.requestFrom(address,count)!=count) return i2cFailed();
  for(uint8_t i=0;i<count;++i) data[i]=Wire.read();
  return true;
}
static void scan() {
  out.print(F("{\"type\":\"i2c\",\"bus\":\"D20/D21\",\"addresses\":["));bool first=true;
  for(uint8_t address=8;address<120;++address) {
    Wire.beginTransmission(address);
    if(Wire.endTransmission()==0) { if(!first) out.print(',');out.print(address);first=false; }
    else i2cFailed();
  }
  out.println(F("],\"note\":\"ACK does not identify a device model\"}"));
}
static int16_t range(uint8_t channel) {
  digitalWrite(trig[channel],LOW);delayMicroseconds(2);
  digitalWrite(trig[channel],HIGH);delayMicroseconds(10);digitalWrite(trig[channel],LOW);
  uint32_t duration=pulseIn(echo[channel],HIGH,18000UL);  // ~3 m: the arena is 1.5 m wide.
  int16_t mm=duration?duration*0.1715f:-1;
  return mm>=20&&mm<=4000?mm:-1;
}

static bool gyroWrite(uint8_t index,uint8_t value) {
  Wire.beginTransmission(gyroAddress);Wire.write(index);Wire.write(value);
  return Wire.endTransmission()==0||i2cFailed();
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
  frames+=drained;if(drained) lastFrameMs=millis();
  return drained;
}
static bool mpuSetup() {
  bool ok=true;
  for(const core::MpuWrite& w:core::MPU_SETUP) ok=gyroWrite(w.reg,w.value)&&ok;
  return gyroWrite(core::MPU_FIFO_EN,core::MPU_FIFO_GYRO_BIT[0]|core::MPU_FIFO_GYRO_BIT[1]|
    core::MPU_FIFO_GYRO_BIT[2])&&ok;
}
// Noise or a supply dip can reset the MPU (back to sleep) or wedge its FIFO. Re-apply the
// setup without the 2 s re-zero: the bias stays valid, the integrated angles do not.
static void gyroConfigure() {
  gyroNeedsSetup=false;lastFrameMs=millis();++gyroRestarts;anglesValid=false;
  if(mpuSetup()) fifoRestart();
}
// The bias is taken only from a still robot: per-axis noise SD under GYRO_STILL_SD raw
// (normally ~2-5). A zero taken while the robot was handled gave a 30 deg/s yaw error.
// Without a valid bias, frames keep accumulating and the robot zeroes itself once still.
static const float GYRO_STILL_SD=25;
static bool biasValid=false;
static float zeroSum[3]={},zeroSquares[3]={};static uint16_t zeroFrames=0;
static void zeroClear() { for(uint8_t a=0;a<3;++a) zeroSum[a]=zeroSquares[a]=0;zeroFrames=0; }
static bool zeroFinish(bool automatic,bool overflowed) {
  float mean[3],sd[3];bool still=zeroFrames>=core::MPU_ZERO_SAMPLES&&!overflowed;
  for(uint8_t a=0;a<3;++a) {
    mean[a]=zeroFrames?zeroSum[a]/zeroFrames:0;
    sd[a]=zeroFrames?sqrt(max(0.0f,zeroSquares[a]/zeroFrames-mean[a]*mean[a])):0;
    still=still&&sd[a]<GYRO_STILL_SD;
  }
  if(still) { for(uint8_t a=0;a<3;++a) { bias[a]=mean[a];biasSd[a]=sd[a];angle[a]=0; } biasValid=anglesValid=true; }
  if(still||!automatic) { // Automatic attempts report only success, not every moving window.
    out.print(F("{\"type\":\"gyro_zero\",\"auto\":"));out.print(automatic);
    out.print(F(",\"frames\":"));out.print(zeroFrames);out.print(F(",\"bias_raw\":["));
    for(uint8_t a=0;a<3;++a) { if(a) out.print(',');out.print(mean[a],2); }
    out.print(F("],\"noise_sd_raw\":["));
    for(uint8_t a=0;a<3;++a) { if(a) out.print(',');out.print(sd[a],2); }
    out.print(F("],\"applied\":"));out.print(still);
    out.println(still?F(",\"valid\":1}"):F(",\"valid\":0,\"note\":\"robot moving: zero rejected, retries by itself when still\"}"));
  }
  zeroClear();return still;
}
static void gyroService() {
  wdt_reset();
  if(!gyroRunning) return;
  if(gyroNeedsSetup||millis()-lastFrameMs>300) gyroConfigure();  // 200 Hz: 300 ms of nothing = stuck.
  bool level=digitalRead(gyroInt);if(level&&!intLast) ++intRises;intLast=level;
  uint32_t now=millis();
  // Data-ready latches INT high; the timer path keeps draining if INT is miswired.
  if(level&&now-lastDrainMs>=(stepKind==STEP_IDLE?20u:5u)) ++intDrains;  // Fresh yaw for step control.
  else if(now-lastDrainMs>=(stepKind==STEP_IDLE?60u:10u)) ++timerDrains;
  else return;
  lastDrainMs=now;
  if(biasValid) gyroDrain(0,0);
  else { zeroFrames+=gyroDrain(zeroSum,zeroSquares);if(zeroFrames>=core::MPU_ZERO_SAMPLES) zeroFinish(true,false); }
}
// GYROZERO and boot: two seconds of frames, applied only if the robot was still.
static void gyroZero() {
  if(!gyroRunning) return;
  zeroClear();fifoRestart();
  uint32_t begin=millis();uint16_t before=overflows;
  while(zeroFrames<core::MPU_ZERO_SAMPLES&&millis()-begin<3000) { wdt_reset();delay(20);zeroFrames+=gyroDrain(zeroSum,zeroSquares); }
  zeroFinish(false,overflows!=before);
}
static void gyroBegin() {
  gyroRunning=anglesValid=biasValid=false;gyroAddress=gyroWho=0;zeroClear();
  pinMode(gyroInt,INPUT);
  // The MPU keeps power through a Mega reset, so a chip left mid-transfer or scrambled
  // can fail setup: retry after a bus recovery, each time with a full device reset.
  for(uint8_t attempt=0;attempt<3&&!gyroRunning;++attempt) {
    wdt_reset();
    if(attempt) i2cRecover();
    gyroAddress=0;
    for(uint8_t address=0x68;address<=0x69&&!gyroAddress;++address)
      if(reg(address,core::MPU_WHO_AM_I,&gyroWho,1)) gyroAddress=address;
    gyroModel=gyroAddress?core::mpuModel(gyroWho):0;
    if(!gyroModel) continue;
    bool ok=gyroWrite(core::MPU_PWR_MGMT_1,0x80);delay(100); // Volatile device reset.
    gyroRunning=mpuSetup()&&ok;
  }
  if(!gyroRunning) return;
  delay(200); // Gyro start-up and DLPF settling before the bias window.
  gyroZero();lastDrainMs=lastFrameMs=millis();
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
  Serial.print(F(",\"i2c_timeouts\":"));Serial.print(i2cTimeouts);
  Serial.print(F(",\"gyro_restarts\":"));Serial.print(gyroRestarts);
  Serial.print(F(",\"gyro_resets\":"));Serial.print(gyroResets);
  Serial.print(F(",\"bias_valid\":"));Serial.print(biasValid);
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
static void compactSample(Print& o,const uint16_t* adc,const int16_t* us,const uint16_t* rgbc) {
  float rate[3];
  for(uint8_t axis=0;axis<3;++axis) rate[axis]=(lastRaw[axis]-bias[axis])/core::MPU_GYRO_LSB_PER_DPS;
  o.print(F("{\"type\":\"sample\",\"link\":\"bt\",\"ms\":"));o.print(millis());
  o.print(F(",\"boot\":\""));o.print(bootCause);o.print('"');
  o.print(F(",\"adc\":"));list(o,adc,4);
  o.print(F(",\"ultrasonic_mm\":"));list(o,us,US_COUNT);
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
  o.print(F(",\"i2c_timeouts\":"));o.print(i2cTimeouts);o.print(F(",\"gyro_restarts\":"));o.print(gyroRestarts);
  o.print(F(",\"gyro_resets\":"));o.print(gyroResets);o.print(F(",\"bias_valid\":"));o.print(biasValid);
  o.print(F("},\"color_rgbc\":"));list(o,rgbc,4);ttReport(o);stepReport(o);o.println('}');
}
// Bluetooth's 9600 baud carries ~960 bytes/s: a full sample (~400 bytes) goes once a second
// and this ~62-byte line fills the other four of five, so the console still updates at 5 Hz.
// F,ms,adc x4,ultrasonic left/right/front,gyro Z angle,gyro Z rate,colour R,G,B,C
static void fastSample(Print& o,const uint16_t* adc,const int16_t* us,const uint16_t* rgbc) {
  o.print(F("F,"));o.print(millis());
  for(uint8_t i=0;i<4;++i) { o.print(',');o.print(adc[i]); }
  for(uint8_t i=0;i<US_COUNT;++i) { o.print(',');o.print(us[i]); }
  o.print(',');o.print(angle[2],1);o.print(',');o.print((lastRaw[2]-bias[2])/core::MPU_GYRO_LSB_PER_DPS,1);
  for(uint8_t i=0;i<4;++i) { o.print(',');o.print(rgbc[i]); }
  o.println();
}
// ---- Tournament step tests ----------------------------------------------------------------
// The console sends one step at a time with targets from the calibration captures. The loop
// runs here: Bluetooth's 5 Hz data arrives 100-300 ms late, too late to stop a turn.
// Ranges: 0 left 45, 1 right 45, 2 front, 3 left side, 4 right side.
//   WMAP m s m s m s m s  wheel map from the console: motor 1-4 and forward sign, FL FR RL RR.
//   REF front ls rs l45 r45  the recorded readings of the target checkpoint; 9999 = the capture
//     had no steady reading there, so that sensor is not used.
//   GO base creep probe maxpwm minpwm  drive until the readings match REF, in the robot's frame:
//     the front sensor sets the distance, the side sensors the capture had set the sideways
//     position. base 0 holds the current heading, 1 the stored reference heading. creep 1 lets
//     the robot creep forward and centre on the 45s until its sensors reach the walls (start).
//     probe 1 then fixes the heading from the recorded 45 readings: it turns +3 and -3 deg,
//     measures how each beam changes per degree (wall geometry unknown) and corrects by
//     (recorded - measured) / slope. On arrival the heading is stored as the reference.
//   TURN deg maxpwm minpwm  gyro turn to the reference heading + deg (CCW +), or the current
//     heading + deg if none is stored; stores that target as the reference.
// Any stop, a 40 s (turn 10 s) timeout, lost sensors or an unzeroed gyro ends a step.
static const float GYRO_SCALE=1.009f;         // Hand test 27 Oct: 4 x 90 deg left read 357.4.
static const float STEP_X_FULL_MM=250,STEP_Y_FULL_MM=150,STEP_YAW_FULL_DEG=25;
static const float STEP_BRAKE_S=0.06f;         // Cut power this long before reaching a heading.
static const float STEP_UNKNOWN=9999;          // Error not measured yet.
static const int16_t STEP_NO_REF=9999;         // Reference the capture did not have.
static const float PROBE_DEG=3,PROBE_MAX_FIX_DEG=8,PROBE_MIN_SLOPE=3; // slope in mm per deg
// Closer than this the beam cone (+/-7.5 deg) finds the same nearest surface whichever way it
// points (e.g. both 45s ~150 mm in a parking corner): no heading information, so not probed.
static const int16_t PROBE_MIN_RANGE_MM=200;
static const uint8_t GO_POS=0,GO_PLUS=1,GO_MINUS=2,GO_ZERO=3,GO_FINAL=4;
static int16_t usLatest[US_COUNT]={-1,-1,-1,-1,-1};static uint16_t usSeq=0; // Set by sample().
static int8_t wheelMotor[4]={-1,-1,-1,-1},wheelSign[4]={};
static int16_t ref[US_COUNT]={STEP_NO_REF,STEP_NO_REF,STEP_NO_REF,STEP_NO_REF,STEP_NO_REF}; // l45 r45 front ls rs
static bool refYawValid=false,goCreep=false,goProbe=false;
static int16_t stepMaxPwm=0,stepMinPwm=0;
static uint8_t goPhase=GO_POS,probesDone=0,probeReads=0;
static float yawTarget=0,refYaw=0,stepEx=0,stepEy=0,goUx=0,goUy=0,probeBase=0,probeFix=0;
static float probeSum[3][2]={},probeN[3][2]={};  // phase PLUS/MINUS/ZERO x beam L/R
static uint32_t stepStartMs=0,stepCtrlMs=0,stepSettledMs=0,stepSensorsOkMs=0;static uint16_t stepUsSeen=0;
static float yawDeg() { return -angle[2]*GYRO_SCALE; }  // CCW positive: the chip's Z points down.
static float yawRateDps() { return -(lastRaw[2]-bias[2])/core::MPU_GYRO_LSB_PER_DPS*GYRO_SCALE; }
static bool hasRef(uint8_t i) { return ref[i]!=STEP_NO_REF; }
static bool probeBeam(uint8_t b) { return hasRef(b)&&ref[b]>=PROBE_MIN_RANGE_MM; }
static bool parseInts(const char* text,int16_t* v,uint8_t n) {
  for(uint8_t i=0;i<n;++i) {
    char* end;long value=strtol(text,&end,10);
    if(end==text||value<-9999||value>9999) return false;
    v[i]=value;text=end;
  }
  while(*text==' ') ++text;
  return !*text;
}
// Mecanum mix, inputs -1..1: ux forward, uy left, uw CCW. A wheel under 3 % stays off; the
// rest map onto minpwm..maxpwm so small corrections still beat friction. All off = hard stop.
static void driveMix(float ux,float uy,float uw) {
  float wheel[4]={ux-uy-uw,ux+uy+uw,ux+uy-uw,ux-uy+uw},peak=1;  // FL FR RL RR
  for(float w:wheel) peak=max(peak,fabsf(w));
  int16_t raw[4]={};bool any=false;
  for(uint8_t p=0;p<4;++p) {
    float u=wheel[p]/peak;
    if(fabsf(u)<0.03f) continue;
    int16_t pwm=int16_t(stepMinPwm+(stepMaxPwm-stepMinPwm)*fabsf(u));
    raw[wheelMotor[p]]+=wheelSign[p]*(u>0?pwm:-pwm);any=true;
  }
  if(!any) { haltMotors();return; }
  for(uint8_t m=0;m<4;++m) setMotor(m,raw[m]);
  motorsLive=true;spinning=-1;motorDeadline=millis()+DRIVE_WATCHDOG_MS;
}
static void stepEnd(bool reached,const __FlashStringHelper* reason) {
  haltMotors();
  if(reached) { refYaw=stepKind==STEP_TURN?yawTarget:yawDeg();refYawValid=true; }
  out.print(F("{\"type\":\"step_end\",\"kind\":\""));out.print(stepKind==STEP_TURN?F("turn"):F("go"));
  out.print(F("\",\"result\":\""));out.print(reached?F("reached"):F("aborted"));
  out.print(F("\",\"reason\":\""));out.print(reason);
  out.print(F("\",\"ex_mm\":"));out.print(stepEx,0);out.print(F(",\"ey_mm\":"));out.print(stepEy,0);
  out.print(F(",\"probes\":"));out.print(probesDone);out.print(F(",\"probe_fix_deg\":"));out.print(probeFix,2);
  out.print(F(",\"yaw_err_deg\":"));out.print(yawTarget-yawDeg(),2);out.print(F(",\"yaw_deg\":"));out.print(yawDeg(),2);
  out.print(F(",\"ref_heading\":"));out.print(refYawValid);out.print(F(",\"ms\":"));out.print(millis());out.println('}');
  stepKind=STEP_IDLE;
}
static bool stepBegin(uint8_t kind,int16_t maxPwm,int16_t minPwm) {
  if(wheelMotor[0]<0) { out.println(F("{\"error\":\"STEP_NEEDS_WMAP\"}"));return false; }
  if(!gyroRunning||!biasValid) { out.println(F("{\"error\":\"STEP_NEEDS_ZEROED_GYRO\"}"));return false; }
  stepKind=kind;stepMaxPwm=constrain(maxPwm,40,int16_t(DRIVE_PWM_MAX));stepMinPwm=constrain(minPwm,0,stepMaxPwm);
  stepStartMs=stepCtrlMs=stepSensorsOkMs=millis();stepSettledMs=0;stepUsSeen=usSeq;
  stepEx=stepEy=STEP_UNKNOWN;goUx=goUy=0;goPhase=GO_POS;probesDone=0;probeFix=0;
  return true;
}
static void stepStarted() {
  out.print(F("{\"type\":\"step_start\",\"kind\":\""));out.print(stepKind==STEP_TURN?F("turn"):F("go"));
  out.print(F("\",\"yaw_target_deg\":"));out.print(yawTarget,2);out.print(F(",\"ref_heading\":"));out.print(refYawValid);
  out.print(F(",\"ms\":"));out.print(millis());out.println('}');
}
// New ranges: position errors and translation commands, in the robot's frame.
static void goUpdate(uint32_t now) {
  int16_t l45=usLatest[0],r45=usLatest[1],front=usLatest[2],ls=usLatest[3],rs=usLatest[4];
  bool okX=hasRef(2)&&front>0;
  stepEx=okX?front-ref[2]:STEP_UNKNOWN;                                 // + = drive forward
  // Sideways (+ = strafe left) from exactly the side sensors the capture had, once all see a wall.
  bool okY=true;
  if(hasRef(3)&&hasRef(4)) { if(ls>0&&rs>0) stepEy=((ls-rs)-(ref[3]-ref[4]))*0.5f;else okY=false; }
  else if(hasRef(4)) { if(rs>0) stepEy=ref[4]-rs;else okY=false; }
  else if(hasRef(3)) { if(ls>0) stepEy=ls-ref[3];else okY=false; }
  else okY=false;
  // Starting out, before a side wall is in range: centre on the 45 pair (beams on the side walls).
  if(!okY&&goCreep&&hasRef(0)&&hasRef(1)&&l45>0&&r45>0) { stepEy=((l45-r45)-(ref[0]-ref[1]))*0.354f;okY=true; }
  if(!okY) stepEy=STEP_UNKNOWN;
  if(okX) stepSensorsOkMs=now;
  goUx=okX?(fabsf(stepEx)>12?constrain(stepEx/STEP_X_FULL_MM,-1.0f,1.0f):0):(goCreep?0.3f:0);
  goUy=okY&&fabsf(stepEy)>8?constrain(stepEy/STEP_Y_FULL_MM,-1.0f,1.0f):0;
  if(front>0&&goUx>0&&(front<30||(okX&&front<ref[2]-20))) goUx=0;    // Never past the target into a wall.
}
static bool positionSettled() {
  bool lateral=hasRef(3)||hasRef(4);
  return goUx==0&&goUy==0&&fabsf(stepEx)<15&&(!lateral||fabsf(stepEy)<10);
}
// Heading probe: average two fresh 45 readings per probe heading, then fit the slopes.
static void probeNext() {
  if(goPhase==GO_PLUS) { yawTarget=probeBase-PROBE_DEG;goPhase=GO_MINUS;return; }
  if(goPhase==GO_MINUS) { yawTarget=probeBase;goPhase=GO_ZERO;return; }
  float fix=0,weight=0;
  out.print(F("{\"type\":\"step_probe\",\"slope_mm_per_deg\":["));
  for(uint8_t b=0;b<2;++b) {
    bool ok=probeBeam(b)&&probeN[0][b]&&probeN[1][b]&&probeN[2][b];
    float slope=ok?(probeSum[0][b]/probeN[0][b]-probeSum[1][b]/probeN[1][b])/(2*PROBE_DEG):0;
    if(b) out.print(',');
    out.print(slope,1);
    if(ok&&fabsf(slope)>=PROBE_MIN_SLOPE) { fix+=(ref[b]-probeSum[2][b]/probeN[2][b])/slope;weight+=1; }
  }
  probeFix=weight?constrain(fix/weight,-PROBE_MAX_FIX_DEG,PROBE_MAX_FIX_DEG):0;
  ++probesDone;yawTarget=probeBase+probeFix;
  out.print(F("],\"fix_deg\":"));out.print(probeFix,2);out.print(F(",\"beams\":"));out.print(int(weight));out.println('}');
  goPhase=fabsf(probeFix)>2&&probesDone<2?GO_POS:GO_FINAL;  // Re-centre; probe again after a big fix.
}
static void stepService() {
  if(stepKind==STEP_IDLE) return;
  uint32_t now=millis();
  if(now-stepCtrlMs<5) return;  // At minimum power the robot still turns ~1 deg per 20 ms.
  stepCtrlMs=now;
  if(now-stepStartMs>(stepKind==STEP_TURN?10000UL:40000UL)) { stepEnd(false,F("timeout"));return; }
  if(!gyroRunning||!biasValid) { stepEnd(false,F("gyro_not_ready"));return; }
  float eYaw=yawTarget-yawDeg(),rate=yawRateDps();
  bool coast=eYaw*rate>0&&fabsf(eYaw)<fabsf(rate)*STEP_BRAKE_S;  // Momentum covers the rest.
  bool headingStill=fabsf(eYaw)<0.6f&&fabsf(rate)<2;
  float uw=headingStill||coast?0:constrain(eYaw/STEP_YAW_FULL_DEG,-1.0f,1.0f);
  bool fresh=usSeq!=stepUsSeen;stepUsSeen=usSeq;
  if(stepKind==STEP_TURN) {  // Slows inside 25 deg; done after 400 ms within 0.6 deg and still.
    driveMix(0,0,uw);
    if(headingStill) { if(!stepSettledMs) stepSettledMs=now;else if(now-stepSettledMs>=400) stepEnd(true,F("reached")); }
    else stepSettledMs=0;
    return;
  }
  if(fresh) goUpdate(now);
  if(now-stepSensorsOkMs>(goCreep?6000UL:1500UL)) { stepEnd(false,F("sensors_lost"));return; }
  if(goPhase==GO_PLUS||goPhase==GO_MINUS||goPhase==GO_ZERO) {  // Turning in place only.
    driveMix(0,0,uw);
    if(!headingStill) { stepSettledMs=0;return; }
    if(!stepSettledMs) { stepSettledMs=now;probeReads=0;return; }
    if(now-stepSettledMs<200||!fresh) return;   // Let the heading settle; ranges taken after.
    uint8_t p=goPhase-GO_PLUS;
    if(!probeReads) for(uint8_t b=0;b<2;++b) probeSum[p][b]=probeN[p][b]=0;
    for(uint8_t b=0;b<2;++b) if(usLatest[b]>0) { probeSum[p][b]+=usLatest[b];probeN[p][b]+=1; }
    if(++probeReads>=2) { stepSettledMs=0;probeNext(); }
    return;
  }
  driveMix(goUx,goUy,fabsf(eYaw)<0.8f||coast?0:constrain(eYaw/STEP_YAW_FULL_DEG,-1.0f,1.0f));
  if(positionSettled()&&fabsf(eYaw)<1.5f) {
    if(!stepSettledMs) { stepSettledMs=now;return; }
    if(now-stepSettledMs<600) return;
    stepSettledMs=0;
    if(goPhase==GO_POS&&goProbe&&(probeBeam(0)||probeBeam(1))) { probeBase=yawTarget;yawTarget=probeBase+PROBE_DEG;goPhase=GO_PLUS; }
    else stepEnd(true,F("reached"));
  } else stepSettledMs=0;
}
static void stepReport(Print& o) {
  o.print(F(",\"step\":{\"kind\":\""));o.print(stepKind==STEP_GO?F("go"):stepKind==STEP_TURN?F("turn"):F("idle"));
  o.print(F("\",\"phase\":"));o.print(goPhase);o.print(F(",\"ref_heading\":"));o.print(refYawValid);
  o.print(F(",\"yaw_deg\":"));o.print(yawDeg(),1);o.print('}');
}
// A scrambled MPU (a write corrupted by noise) reads a wrong ID or impossible gravity, such
// as -32768 on every accel axis, while the bus itself works; only a full device reset cures
// it. Two bad checks in a row (0.5 s) trigger one. The bias belongs to the chip and is kept.
static uint8_t gyroBadChecks=0;
static void gyroHealth() {
  if(!gyroRunning) return;
  const float LSB_PER_G=16384;  // ACCEL_CONFIG is left at its reset value, +/-2 g.
  float g2=0;for(uint8_t a=0;a<3;++a) g2+=float(accelRaw[a])*accelRaw[a];
  uint8_t who=0;
  bool healthy=reg(gyroAddress,core::MPU_WHO_AM_I,&who,1)&&who==gyroWho&&accelOk&&
    g2>sq(0.5f*LSB_PER_G)&&g2<sq(1.6f*LSB_PER_G);
  if(healthy) { gyroBadChecks=0;return; }
  if(++gyroBadChecks<2) return;
  gyroBadChecks=0;++gyroResets;anglesValid=false;
  gyroWrite(core::MPU_PWR_MGMT_1,0x80);wdt_reset();delay(100);
  if(mpuSetup()) fifoRestart();
  lastFrameMs=millis();
}
static void service(); // Commands, motor watchdog and gyro, also called mid-sample.
static bool sampling=false;
static void sample() {
  if(sampling) return;
  sampling=true;
  uint32_t cycleStart=millis();
  if(!motorsLive) inhibitMotors();
  uint16_t value[4]={};
  for(uint8_t i=0;i<4;++i) { analogRead(light[i]);for(uint8_t n=0;n<16;++n) value[i]+=analogRead(light[i]);value[i]=(value[i]+8)/16; }
  int16_t us[US_COUNT];
  for(uint8_t i=0;i<US_COUNT;++i) {
    if(i) for(uint32_t t=millis();millis()-t<15;) service();
    us[PING_ORDER[i]]=range(PING_ORDER[i]);
  }
  for(uint8_t i=0;i<US_COUNT;++i) usLatest[i]=us[i];
  ++usSeq;service();
  // Channels R, G, B, clear; 3 ms filter settling, then 96 bursts (6.912 ms exposure).
  // Skipped during a step so ranges update faster; the last reading is repeated.
  static uint16_t rgbc[4]={};
  for(uint8_t channel=0;stepKind==STEP_IDLE&&channel<4;++channel) {
    if(!channel) for(uint16_t& c:rgbc) c=0;
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
  Serial.print(F("],\"ultrasonic_mm\":"));list(Serial,us,US_COUNT);
  Serial.print(F(",\"gyro\":"));gyroService();readAccel();gyroReport(); // No output mid-line.
  Serial.print(F(",\"color_out_pin\":32,\"color_rgbc\":["));
  for(uint8_t i=0;i<4;++i) { if(i) Serial.print(',');Serial.print(rgbc[i]); }
  Serial.print(']');
  ttReport(Serial);stepReport(Serial);
  Serial.print(F(",\"color_exposure_cycles\":"));Serial.print(uint32_t(core::COLOR_MIN_BURSTS)*core::COLOR_BURST_CYCLES);
  Serial.print(F(",\"color_filters\":\"DRIVEN_20PCT\",\"cycle_ms\":"));Serial.print(millis()-cycleStart);Serial.println('}');
  static uint32_t lastBtMs=0;  // Steps sample at 10 Hz; Bluetooth stays at 5 Hz, its limit.
  if(btHostPresent()&&millis()-lastBtMs>=190) { lastBtMs=millis();if(++sampleCount%5==0) compactSample(Serial3,value,us,rgbc);else fastSample(Serial3,value,us,rgbc); }
  gyroHealth();
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
  if(!strncmp(line,"WMAP ",5)) {
    int16_t v[8];bool ok=parseInts(line+5,v,8);uint8_t used=0;
    for(uint8_t p=0;ok&&p<4;++p) { ok=v[2*p]>=1&&v[2*p]<=4&&(v[2*p+1]==1||v[2*p+1]==-1)&&!(used&(1<<v[2*p]));used|=1<<v[2*p]; }
    if(!ok) { out.println(F("{\"error\":\"BAD_WMAP\"}"));return; }
    for(uint8_t p=0;p<4;++p) { wheelMotor[p]=v[2*p]-1;wheelSign[p]=v[2*p+1]; }
    out.println(F("{\"type\":\"wmap\",\"ok\":1}"));
    return;
  }
  if(!strncmp(line,"REF ",4)) {
    int16_t v[5];
    if(!parseInts(line+4,v,5)) { out.println(F("{\"error\":\"BAD_REF\"}"));return; }
    const uint8_t order[5]={2,3,4,0,1};  // front ls rs l45 r45 -> range indices
    for(uint8_t i=0;i<5;++i) ref[order[i]]=v[i]>0?v[i]:STEP_NO_REF;
    out.println(F("{\"type\":\"ref\",\"ok\":1}"));
    return;
  }
  if(!strncmp(line,"GO ",3)) {
    int16_t v[5];stopMotors();
    if(!parseInts(line+3,v,5)||v[0]<0||v[0]>1||v[1]<0||v[1]>1||v[2]<0||v[2]>1) { out.println(F("{\"error\":\"BAD_GO_STOPPED\"}"));return; }
    if(!hasRef(2)&&!v[1]) { out.println(F("{\"error\":\"GO_NEEDS_FRONT_REF\"}"));return; }
    if(!stepBegin(STEP_GO,v[3],v[4])) return;
    goCreep=v[1];goProbe=v[2];
    yawTarget=v[0]&&refYawValid?refYaw:yawDeg();
    stepStarted();
    return;
  }
  if(!strncmp(line,"TURN ",5)) {
    int16_t v[3];stopMotors();
    if(!parseInts(line+5,v,3)||v[0]<-180||v[0]>180) { out.println(F("{\"error\":\"BAD_TURN_STOPPED\"}"));return; }
    if(!stepBegin(STEP_TURN,v[1],v[2])) return;
    yawTarget=(refYawValid?refYaw:yawDeg())+v[0];
    stepStarted();
    return;
  }
  if(!strncmp(line,"TURNTEST ",9)) {
    int16_t v[4];stopMotors();
    if(!parseDrive(line+9,v)||!(v[0]|v[1]|v[2]|v[3])) { out.println(F("{\"error\":\"BAD_TURNTEST_STOPPED\"}"));return; }
    if(!gyroRunning) { out.println(F("{\"error\":\"TURNTEST_NEEDS_GYRO\"}"));return; }
    for(uint8_t m=0;m<4;++m) ttPwm[m]=v[m];
    ttLeg=0;ttLegsDone=0;ttState=TT_RUNNING;ttStartLeg();
    out.print(F("{\"type\":\"turntest_start\",\"legs\":"));out.print(TT_LEGS);
    out.print(F(",\"deg\":"));out.print(TT_DEG,0);out.print(F(",\"ms\":"));out.print(millis());out.println('}');
    return;
  }
  if(!strncmp(line,"MOT ",4)) {
    int16_t v[4];
    if(!parseDrive(line+4,v)) { stopMotors();out.println(F("{\"error\":\"BAD_MOT_STOPPED\"}"));return; }
    if(spinning>=0||ttLeg>=0||stepKind!=STEP_IDLE) stopMotors(); // Manual drive cancels a test or step.
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
  inhibitMotors();Serial.begin(115200);Serial3.begin(BLUETOOTH_BAUD);i2cBegin();
  for(uint8_t i=0;i<US_COUNT;++i) { digitalWrite(trig[i],LOW);pinMode(trig[i],OUTPUT);pinMode(echo[i],INPUT); }
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
  ttService();
  stepService();
  rampMotors();
  if(motorsLive&&int32_t(millis()-motorDeadline)>=0) {
    int8_t m=spinning;stopMotors();
    if(m>=0) { out.print(F("{\"type\":\"spin_done\",\"motor\":"));out.print(m+1);out.println('}'); }
    else out.println(F("{\"type\":\"drive_timeout_stop\"}"));
  }
}
void loop() {
  service();
  if(millis()-lastSample>=(stepKind==STEP_IDLE?200u:100u)) { lastSample=millis();sample(); }  // 5 Hz, 10 Hz in a step
}
