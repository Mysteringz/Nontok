#include "config.h"
#include "sensors.h"
#include <Wire.h>
#include <math.h>
#include "frequency_color.h"
#include "mpu_gyro.h"
static core::Sensors readings;
#if SENSOR_WIRING_CONFIRMED
// Gyro supervision, ported from the 27 Oct hardware probe. The bias is taken only from a
// still robot: a zero averaged while the robot was handled once gave a 30 deg/s yaw error.
// imuValid stays false until then, which blocks START, captures and localization.
// A scrambled chip (a write corrupted by noise) reads a wrong ID or impossible gravity
// while the bus works; two bad checks in a row (0.5 s) trigger a full chip reset.
enum class Imu : uint8_t { ABSENT, SETUP, ZEROING, READY };
static Imu imuState=Imu::ABSENT;
static const char* gyroState="GYRO_ABSENT";
static bool calibrated=false;   // bias is valid; it belongs to the chip and survives a reset
static float bias=0;
static uint32_t imuAt=0,lastHealthMs=0;
static int16_t zeroRef=0;static int32_t zeroSum=0;static float zeroSquares=0;
static uint16_t zeroCount=0;static uint8_t zeroSkip=0,imuBadChecks=0;
const float GYRO_STILL_DPS=0.4f;   // per-sample noise SD; a still robot measures ~0.03
const float ACCEL_LSB_PER_G=16384.0f; // ACCEL_CONFIG left at reset, +/-2 g
// Every 200 Hz Z sample goes through the chip FIFO (2 bytes each) and is integrated, so
// heading no longer depends on loop timing; ultrasonic echo waits used to hide motion.
const uint8_t FIFO_CHUNK=30;       // Wire buffers 32 bytes: 15 samples per read.
static uint32_t lastImuMs=0,lastPingMs=0,lastLightMs=0,lastColorMs=0;
static uint32_t rangeAt[2]={};
static uint8_t nextRange=0,sampleIndex[2]={},sampleCount[2]={};
static int16_t history[2][3]={};
// After a timeout, free a bus a slave holds mid-byte: up to 9 SCL clocks, then STOP.
static void i2cRecover() {
  Wire.end();pinMode(SDA,INPUT_PULLUP);pinMode(SCL,INPUT_PULLUP);  // Open drain by hand.
  for(uint8_t i=0;i<9&&!digitalRead(SDA);++i) {
    digitalWrite(SCL,LOW);pinMode(SCL,OUTPUT);delayMicroseconds(5);
    pinMode(SCL,INPUT_PULLUP);delayMicroseconds(5);
  }
  digitalWrite(SDA,LOW);pinMode(SDA,OUTPUT);delayMicroseconds(5);  // SDA rises with SCL high.
  pinMode(SDA,INPUT_PULLUP);delayMicroseconds(5);
  Wire.begin();Wire.setWireTimeout(25000,true);
}
static bool i2cFailed() {  // After any failed transfer: only a timeout means a stuck bus.
  if(Wire.getWireTimeoutFlag()) { Wire.clearWireTimeoutFlag();i2cRecover(); }
  return false;
}
static bool writeReg(uint8_t address,uint8_t reg,uint8_t value) {
  Wire.beginTransmission(address);Wire.write(reg);Wire.write(value);return Wire.endTransmission()==0||i2cFailed();
}
static bool readReg(uint8_t address,uint8_t reg,uint8_t* bytes,uint8_t count) {
  Wire.beginTransmission(address);Wire.write(reg);
  if(Wire.endTransmission(false)!=0||Wire.requestFrom(address,count)!=count) return i2cFailed();
  for(uint8_t i=0;i<count;++i) bytes[i]=Wire.read();
  return true;
}
// Identify, then a full chip reset; setup follows 100 ms later without blocking the loop.
// The MPU keeps power through a Mega reset, so it may still be mid-transfer or scrambled.
static void imuStart(uint32_t now) {
  uint8_t identity;readings.imuValid=false;
  if(readReg(IMU_ADDRESS,0x75,&identity,1)&&(identity&0x7e)==0x68&&writeReg(IMU_ADDRESS,0x6b,0x80)) {
    imuState=Imu::SETUP;imuAt=now+100;gyroState="GYRO_SETUP";
  } else { imuState=Imu::ABSENT;imuAt=now+500;gyroState="GYRO_ABSENT_RETRYING"; }
}
static bool fifoRestart() {
  return writeReg(IMU_ADDRESS,core::MPU_USER_CTRL,core::MPU_USER_FIFO_RESET)&&
         writeReg(IMU_ADDRESS,core::MPU_USER_CTRL,core::MPU_USER_FIFO_ON);
}
// PLL clock, 200 Hz, ~44 Hz DLPF, +/-500 deg/s, Z gyro only into the FIFO.
static void imuSetup(uint32_t now) {
  if(!writeReg(IMU_ADDRESS,core::MPU_PWR_MGMT_1,0x01)||!writeReg(IMU_ADDRESS,core::MPU_SMPLRT_DIV,core::MPU_SAMPLE_DIVIDER)||
     !writeReg(IMU_ADDRESS,core::MPU_CONFIG,0x03)||!writeReg(IMU_ADDRESS,core::MPU_GYRO_CONFIG,0x08)||
     !writeReg(IMU_ADDRESS,core::MPU_FIFO_EN,core::MPU_FIFO_GYRO_BIT[2])||!fifoRestart()) { imuStart(now);return; }
  lastImuMs=now;imuBadChecks=0;
  if(calibrated) { imuState=Imu::READY;gyroState="GYRO_READY";return; }
  imuState=Imu::ZEROING;zeroCount=0;zeroSkip=20;gyroState="GYRO_ZEROING_HOLD_STILL"; // Skip DLPF settling.
}
// 2 s windows; a window with motion is discarded and the next one starts at once.
static void zeroAdd(int16_t raw) {
  if(zeroSkip) { --zeroSkip;return; }
  if(!zeroCount) { zeroRef=raw;zeroSum=0;zeroSquares=0; }
  int32_t d=int32_t(raw)-zeroRef;zeroSum+=d;zeroSquares+=float(d)*d; // Offsets keep floats exact.
  if(++zeroCount<core::MPU_ZERO_SAMPLES) return;
  float mean=float(zeroSum)/zeroCount,sd=sqrtf(fmaxf(0.0f,zeroSquares/zeroCount-mean*mean));
  zeroCount=0;
  if(sd/core::MPU_GYRO_LSB_PER_DPS>GYRO_STILL_DPS) return;
  bias=zeroRef+mean;calibrated=true;readings.heading=0;imuState=Imu::READY;gyroState="GYRO_ZEROED";
}
// Integrate every buffered sample. A FIFO near full means samples were lost: restart it and
// report the heading invalid for this update. At most 4 reads (60 samples) per call.
static void imuDrain(uint32_t now) {
  uint8_t b[FIFO_CHUNK];
  if(!readReg(IMU_ADDRESS,core::MPU_FIFO_COUNT,b,2)) { readings.imuValid=false;return; }
  uint16_t bytes=(uint16_t(b[0])<<8)|b[1];
  if(bytes>=core::MPU_FIFO_BYTES_MIN) { fifoRestart();readings.imuValid=false;lastImuMs=now;return; }
  bytes&=~1u;float rateSum=0;uint16_t samples=0;
  for(uint8_t reads=0;bytes&&reads<4;++reads) {
    uint8_t chunk=bytes>FIFO_CHUNK?FIFO_CHUNK:bytes;
    if(!readReg(IMU_ADDRESS,core::MPU_FIFO_RW,b,chunk)) { fifoRestart();readings.imuValid=false;return; } // Alignment lost.
    for(uint8_t at=0;at<chunk;at+=2) {
      int16_t raw=core::mpuWord(b+at);
      if(imuState==Imu::ZEROING) { zeroAdd(raw);if(imuState==Imu::READY) rateSum=samples=0;continue; }
      float rate=GYRO_SIGN*(raw-bias)/core::MPU_GYRO_LSB_PER_DPS;
      readings.heading+=rate/core::MPU_SAMPLE_HZ;rateSum+=rate;++samples;
    }
    bytes-=chunk;
  }
  lastImuMs=now;
  if(imuState==Imu::ZEROING) { readings.imuValid=false;return; }
  if(samples) readings.gyroRate=rateSum/samples;
  readings.imuValid=true;
}
static void imuHealth(uint32_t now) {
  uint8_t identity,b[6];float g2=0;
  bool ok=readReg(IMU_ADDRESS,0x75,&identity,1)&&(identity&0x7e)==0x68&&readReg(IMU_ADDRESS,0x3b,b,6);
  for(uint8_t i=0;ok&&i<3;++i) { float a=(int16_t)(((uint16_t)b[2*i]<<8)|b[2*i+1]);g2+=a*a; }
  const float low=0.5f*ACCEL_LSB_PER_G,high=1.6f*ACCEL_LSB_PER_G;
  if(ok&&g2>low*low&&g2<high*high) { imuBadChecks=0;return; }
  if(++imuBadChecks<2) return;
  imuStart(now);if(imuState==Imu::SETUP) gyroState="GYRO_RESET_IMPLAUSIBLE";
}
#if COLOR_DRIVER == 1
static bool colorReady=false;
#endif
#if COLOR_DRIVER == 2
#if !defined(__AVR_ATmega2560__)
#error "The frequency sampler requires ATmega2560 D32/PC5"
#endif
static_assert(COLOR_OUT==32,"Direct-port sampler input is D32/PC5, not an arbitrary GPIO");
static_assert(F_CPU==core::COLOR_CPU_HZ,"Colour sampling timing requires a 16 MHz Mega");
extern "C" uint8_t colorPulseBurst();
static core::FrequencyColor frequencyColor;
static core::FrequencyWindow sampleColor() {
  core::FrequencyWindow window;
  for(uint16_t burst=0;burst<core::COLOR_MAX_BURSTS;++burst) {
    uint8_t status=SREG;cli();
    uint8_t edges=colorPulseBurst();
    SREG=status; // Restore interrupt state between every ~73 us burst.
    window.edges+=edges;window.cycles+=core::COLOR_BURST_CYCLES;
    if(burst+1>=core::COLOR_MIN_BURSTS&&window.edges>=64) break;
    // Vary the unobserved gaps to avoid repeatedly sampling the same pulse phase.
    delayMicroseconds((burst*13U)&31U);
  }
  return window;
}
static void selectColor(uint8_t channel) {
  digitalWrite(COLOR_S2,core::frequencyS2(channel)?HIGH:LOW);
  digitalWrite(COLOR_S3,core::frequencyS3(channel)?HIGH:LOW);
}
#endif
#endif
void sensorsBegin() {
  Wire.begin();
  // Unconditional: this framework's Wire never defines WIRE_HAS_TIMEOUT, and without a
  // timeout motor noise on SDA/SCL hangs a transfer forever. The limit covers a whole
  // transfer (32 bytes take ~3 ms at 100 kHz), so 2.5 ms would fail healthy reads.
  Wire.setWireTimeout(25000,true);
#if SENSOR_WIRING_CONFIRMED
  for(uint8_t i=0;i<2;++i) { pinMode(RANGE_TRIG[i],OUTPUT);pinMode(RANGE_ECHO[i],INPUT);digitalWrite(RANGE_TRIG[i],LOW); }
#if LIGHT_DRIVER == 1
  for(uint8_t pin:LIGHT_PIN) pinMode(pin,INPUT);
#endif
#if COLOR_DRIVER == 1
  uint8_t colorIdentity;
  colorReady=readReg(COLOR_ADDRESS,0x92,&colorIdentity,1)&&(colorIdentity==0x44||colorIdentity==0x4d);
  if(colorReady) {
    colorReady=writeReg(COLOR_ADDRESS,0x80,1)&&writeReg(COLOR_ADDRESS,0x81,0xeb)&&writeReg(COLOR_ADDRESS,0x8f,1);
    delay(3);colorReady=writeReg(COLOR_ADDRESS,0x80,3)&&colorReady; // 50.4 ms, gain 4x.
  }
#elif COLOR_DRIVER == 2
  pinMode(COLOR_S2,OUTPUT);pinMode(COLOR_S3,OUTPUT);pinMode(COLOR_OUT,INPUT);
#if COLOR_SCALE_PINS_EXPOSED
  pinMode(COLOR_S0,OUTPUT);pinMode(COLOR_S1,OUTPUT);
  digitalWrite(COLOR_S0,HIGH);digitalWrite(COLOR_S1,LOW);
#endif
  // D47 belongs to ultrasonic ECHO. Sampling D32 leaves all timers untouched.
  selectColor(0);frequencyColor.begin(micros());
#endif
  imuStart(millis());
#endif
}
void sensorsUpdate() {
#if SENSOR_WIRING_CONFIRMED
  uint32_t now=millis();
  if(imuState==Imu::ABSENT||imuState==Imu::SETUP) {
    if(int32_t(now-imuAt)>=0) { if(imuState==Imu::ABSENT) imuStart(now);else imuSetup(now); }
  } else if(now-lastImuMs>=5) {
    imuDrain(now);
    if(now-lastHealthMs>=250) { lastHealthMs=now;imuHealth(now); }
  }
#if LIGHT_DRIVER == 1
  if(now-lastLightMs>=20) {
    for(uint8_t i=0;i<3;++i) { int raw=analogRead(LIGHT_PIN[i]);readings.light[i]=LIGHT_HIGH_IS_BRIGHT?raw:1023-raw; }
    readings.lightValid=true;++readings.lightSequence;lastLightMs=now;
  }
#endif
  if(now-lastPingMs>=40) {
    uint8_t i=nextRange;nextRange=1-nextRange;
    digitalWrite(RANGE_TRIG[i],LOW);delayMicroseconds(2);digitalWrite(RANGE_TRIG[i],HIGH);delayMicroseconds(10);digitalWrite(RANGE_TRIG[i],LOW);
    unsigned long duration=pulseIn(RANGE_ECHO[i],HIGH,24000UL);
    int16_t mm=(int16_t)(duration*0.1715f);
    if(duration==0||mm<20||mm>4000) { readings.range[i]=-1;sampleCount[i]=0; }
    else {
      history[i][sampleIndex[i]]=mm;sampleIndex[i]=(sampleIndex[i]+1)%3;
      if(sampleCount[i]<3) ++sampleCount[i];
      if(sampleCount[i]<3) readings.range[i]=mm;
      else { int16_t a=history[i][0],b=history[i][1],c=history[i][2];readings.range[i]=a>b?(b>c?b:(a>c?c:a)):(a>c?a:(b>c?c:b)); }
    }
    ++readings.rangeSequence[i];rangeAt[i]=millis();lastPingMs=rangeAt[i];
  }
#if COLOR_DRIVER == 1
  if(colorReady&&now-lastColorMs>=65) {
    uint8_t status,b[8];
    readings.colorValid=readReg(COLOR_ADDRESS,0x93,&status,1)&&(status&1)&&readReg(COLOR_ADDRESS,0xb4,b,8);
    if(readings.colorValid) {
      readings.rgba[3]=uint16_t(b[0])|(uint16_t(b[1])<<8);
      for(uint8_t i=0;i<3;++i) readings.rgba[i]=uint16_t(b[2+i*2])|(uint16_t(b[3+i*2])<<8);
      if(readings.rgba[3]>=16128) readings.colorValid=false;
      ++readings.colorSequence;
    }
    lastColorMs=millis();
  }
#elif COLOR_DRIVER == 2
  if(frequencyColor.ready(micros())) {
    core::FrequencyWindow window=sampleColor();
    core::FrequencyStep step=frequencyColor.update(micros(),window);
    if(step.frameReady) {
      readings.colorValid=frequencyColor.valid();
      if(readings.colorValid) {
        for(uint8_t i=0;i<4;++i) readings.rgba[i]=frequencyColor.value(i);
        ++readings.colorSequence;
      }
      lastColorMs=millis();
    }
    if(step.filterChanged) selectColor(frequencyColor.channel());
  }
#endif
  uint32_t completed=millis();
  for(uint8_t i=0;i<2;++i) if(completed-rangeAt[i]>350) readings.range[i]=-1;
  if(completed-lastImuMs>100) readings.imuValid=false;
  if(completed-lastLightMs>150) readings.lightValid=false;
  if(completed-lastColorMs>300) readings.colorValid=false;
#endif
}
core::Sensors sensorsRead() { return readings; }
const char* sensorsGyroState() {
#if SENSOR_WIRING_CONFIRMED
  return gyroState;
#else
  return "GYRO_DISABLED_SENSOR_WIRING_UNCONFIRMED";
#endif
}
void sensorsProbe(Stream& out) {
  out.println(F("# PROBE I2C addresses only; ACK is not a confirmed model"));
  for(uint8_t address=8;address<120;++address) {
    Wire.beginTransmission(address);
    if(Wire.endTransmission()==0) { out.print(F("# I2C "));out.println(address); }
  }
  out.println(F("# PROBE END; analog/frequency modules need physical identification"));
}
