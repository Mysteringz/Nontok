#include "config.h"
#include "sensors.h"
#include <Wire.h>
#include <math.h>
#include "frequency_color.h"
static core::Sensors readings;
#if SENSOR_WIRING_CONFIRMED
static bool calibrated=false;
static float bias=0;
static uint32_t lastImuUs=0,lastImuMs=0,lastPingMs=0,lastLightMs=0,lastColorMs=0;
static uint32_t rangeAt[2]={};
static uint8_t nextRange=0,sampleIndex[2]={},sampleCount[2]={};
static int16_t history[2][3]={};
static bool writeReg(uint8_t address,uint8_t reg,uint8_t value) {
  Wire.beginTransmission(address);Wire.write(reg);Wire.write(value);return Wire.endTransmission()==0;
}
static bool readReg(uint8_t address,uint8_t reg,uint8_t* bytes,uint8_t count) {
  Wire.beginTransmission(address);Wire.write(reg);
  if(Wire.endTransmission(false)!=0||Wire.requestFrom(address,count)!=count) return false;
  for(uint8_t i=0;i<count;++i) bytes[i]=Wire.read();
  return true;
}
static bool rawGyro(int16_t& value) {
  uint8_t b[2];if(!readReg(IMU_ADDRESS,0x47,b,2)) return false;
  value=(int16_t)(((uint16_t)b[0]<<8)|b[1]);return true;
}
#if COLOR_DRIVER == 1
static bool colorReady=false;
#endif
#if COLOR_DRIVER == 2
#if !defined(__AVR_ATmega2560__)
#error "The frequency sensor counter requires ATmega2560 Timer5 and OUT on D47/T5"
#endif
static_assert(COLOR_OUT==47,"Timer5 external clock input is D47/T5, not an arbitrary GPIO");
static core::FrequencyColor frequencyColor;
static void selectColor(uint8_t channel) {
  digitalWrite(COLOR_S2,core::frequencyS2(channel)?HIGH:LOW);
  digitalWrite(COLOR_S3,core::frequencyS3(channel)?HIGH:LOW);
}
#endif
#endif
void sensorsBegin() {
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(2500,true);
#endif
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
  // T5 counts edges in hardware; even 100% scaling does not create per-edge ISRs.
  TCCR5A=0;TCCR5B=0;TCCR5C=0;TIMSK5=0;TCNT5=0;
  TIFR5=_BV(TOV5)|_BV(OCF5A)|_BV(OCF5B)|_BV(OCF5C)|_BV(ICF5);
  TCCR5B=_BV(CS52)|_BV(CS51)|_BV(CS50); // External T5 clock, rising edge.
  selectColor(0);frequencyColor.begin(micros());
#endif
  uint8_t identity;
  if(!readReg(IMU_ADDRESS,0x75,&identity,1)||(identity&0x7e)!=0x68) return;
  if(!writeReg(IMU_ADDRESS,0x6b,0)||!writeReg(IMU_ADDRESS,0x1b,0)||!writeReg(IMU_ADDRESS,0x1a,3)) return;
  delay(100);int32_t sum=0;
  for(uint16_t i=0;i<250;++i) { int16_t raw;if(!rawGyro(raw)) return;sum+=raw;delay(2); }
  bias=sum/250.0f;calibrated=true;lastImuUs=micros();lastImuMs=millis();
#endif
}
void sensorsUpdate() {
#if SENSOR_WIRING_CONFIRMED
  uint32_t now=millis();
  if(calibrated&&now-lastImuMs>=10) {
    int16_t raw;uint32_t us=micros();float dt=(us-lastImuUs)*1e-6f;
    readings.imuValid=rawGyro(raw)&&dt<=0.1f;
    if(readings.imuValid) { readings.gyroRate=GYRO_SIGN*(raw-bias)/131.0f;readings.heading+=readings.gyroRate*dt; }
    lastImuUs=us;lastImuMs=now;
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
  // The sample time is immediately adjacent to the hardware-counter read.
  // No timer ISR touches TCNT5; AVR low-byte reads latch its high byte.
  uint32_t sampleUs=micros();uint16_t counter=TCNT5;
  core::FrequencyStep step=frequencyColor.update(sampleUs,counter);
  if(step.frameReady) {
    readings.colorValid=frequencyColor.valid();
    if(readings.colorValid) {
      for(uint8_t i=0;i<4;++i) readings.rgba[i]=frequencyColor.value(i);
      ++readings.colorSequence;
    }
    lastColorMs=millis();
  }
  if(step.filterChanged) selectColor(frequencyColor.channel());
#endif
  uint32_t completed=millis();
  for(uint8_t i=0;i<2;++i) if(completed-rangeAt[i]>350) readings.range[i]=-1;
  if(completed-lastImuMs>100) readings.imuValid=false;
  if(completed-lastLightMs>150) readings.lightValid=false;
  if(completed-lastColorMs>300) readings.colorValid=false;
#endif
}
core::Sensors sensorsRead() { return readings; }
void sensorsProbe(Stream& out) {
  out.println(F("# PROBE I2C addresses only; ACK is not a confirmed model"));
  for(uint8_t address=8;address<120;++address) {
    Wire.beginTransmission(address);
    if(Wire.endTransmission()==0) { out.print(F("# I2C "));out.println(address); }
  }
  out.println(F("# PROBE END; analog/frequency modules need physical identification"));
}
