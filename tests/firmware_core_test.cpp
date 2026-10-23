#include "support/Arduino.h"
#include "support/EEPROM.h"
#include "control.h"
#include "calibration.h"
#include <cassert>
#include <iostream>
#include <limits>
#include <algorithm>
uint32_t fakeMs=0;
int pwmValues[80]={},directionValues[80]={},pinWrites=0;
Stream Serial,Serial2,Serial3;
FakeEEPROM EEPROM;
static core::Sensors mockSensors;
void sensorsBegin() {}
void sensorsUpdate() {}
core::Sensors sensorsRead() { return mockSensors; }
void sensorsProbe(Stream& s) { s.println("# PROBE END"); }
void displayBegin() {}
void displayUpdate(const core::Controller&,const core::Sensors&) {}
#include "../firmware/core_robot_mega/src/main.cpp"
static float output[3];
static void capture(float x,float y,float z) { output[0]=x;output[1]=y;output[2]=z; }
static core::Profile goodProfile() {
  core::Profile p;core::defaultProfile(p);p.beaconPositionMask=7; // Explicit synthetic collinear fixture.
  for(auto& b:p.beacon) { b.ambient=10;b.signal=50;b.distance=1000; }
  p.colorMask=3;
  float c[2][3]={{0.7f,0.2f,0.1f},{0.2f,0.7f,0.1f}};
  memcpy(p.color,c,sizeof(c));
  for(uint8_t i=0;i<core::POINTS;++i) {
    auto& r=p.point[i];r.x=300;r.y=i==core::BOX_LEFT?-400:i==core::BOX_RIGHT?400:0;
    r.yaw=i==core::TURN_LEFT||i==core::BOX_LEFT?-90:i==core::TURN_RIGHT||i==core::BOX_RIGHT?90:180;
    r.range[0]=400;r.range[1]=500;r.samples=15;
    r.jacobian[0][0]=0.7f;r.jacobian[0][1]=0.7f;r.jacobian[0][2]=3;
    r.jacobian[1][0]=0.7f;r.jacobian[1][1]=-0.7f;r.jacobian[1][2]=-3;
  }
  return p;
}
static core::Sensors goodSensors() {
  core::Sensors s;s.range[0]=400;s.range[1]=500;s.imuValid=s.lightValid=s.colorValid=true;
  s.rgba[0]=200;s.rgba[1]=700;s.rgba[2]=100;s.rgba[3]=1000;return s;
}
static core::Pose referencePose(const core::Profile& p,uint8_t id) {
  core::Pose v;v.x=p.point[id].x;v.y=p.point[id].y;v.yaw=p.point[id].yaw;
  v.valid=v.headingValid=v.ultrasonic=true;v.reference=id;return v;
}
static void advance(core::Controller& c,core::Sensors& s,core::Pose& p,uint32_t& t,uint32_t ms,bool freshColor=true) {
  for(uint32_t elapsed=0;elapsed<ms;elapsed+=20) { t+=20;if(freshColor&&elapsed%80==0) ++s.colorSequence;c.tick(t,s,p); }
}
static void send(const std::string& text,Stream& stream=Serial) {
  for(char c:text) stream.input.push_back(c);stream.input.push_back('\n');
  while(stream.available()) loop();
}
static void mission(bool green) {
  auto p=goodProfile();auto s=goodSensors();auto pose=referencePose(p,core::CENTER);
  if(!green) std::swap(s.rgba[0],s.rgba[1]);
  core::Controller c(capture,true,true,p);assert(c.arm());uint32_t t=0;assert(c.start(t,s,pose));
  pose.x+=200;advance(c,s,pose,t,20);assert(output[0]>0); // World approach from +X, facing 180.
  pose=referencePose(p,core::CENTER);advance(c,s,pose,t,740);assert(c.mode()==core::Mode::READ_COLOR);
  advance(c,s,pose,t,500,false);assert(c.mode()==core::Mode::READ_COLOR); // Five loops are not five fresh samples.
  advance(c,s,pose,t,400);assert(c.mode()==core::Mode::TURN);
  assert(c.decision()==(green?core::Color::GREEN:core::Color::RED));
  advance(c,s,pose,t,20);assert(green?output[2]>0:output[2]<0);
  uint8_t turn=green?core::TURN_LEFT:core::TURN_RIGHT;
  s.heading=green?90:-90;pose=referencePose(p,turn);pose.ultrasonic=false;
  advance(c,s,pose,t,700);assert(c.mode()==core::Mode::TURN); // Gyro alone cannot finish the turn.
  pose.ultrasonic=true;advance(c,s,pose,t,640);assert(c.mode()==core::Mode::TO_BOX);
  pose=referencePose(p,green?core::BOX_LEFT:core::BOX_RIGHT);advance(c,s,pose,t,1040);
  assert(c.mode()==core::Mode::FINISHED&&!c.armed());assert(output[0]==0&&output[1]==0&&output[2]==0);
  assert(!c.arm()&&!c.start(t,s,pose)&&!c.move(10,0,0,500,t));
  advance(c,s,pose,t,4000);assert(c.mode()==core::Mode::FINISHED);c.stop();assert(c.arm());
}
int main() {
  auto p=goodProfile();assert(core::profileValid(p));
  mission(true);mission(false);
  {
    core::Profile unset;core::defaultProfile(unset);assert(unset.beaconPositionMask==2);
    assert(!core::beaconGeometryValid(unset)&&!core::profileValid(unset));
    auto optical=goodSensors();optical.light[0]=optical.light[1]=optical.light[2]=500;
    core::Localizer unknown;assert(!unknown.update(optical,unset).valid);
    auto measured=p;measured.beacon[0].x=180;measured.beacon[2].x=240; // Test fixture, not measured field values.
    assert(core::beaconGeometryValid(measured)&&core::profileValid(measured));
    for(const auto& xy : {std::pair<float,float>(300,0),{650,120},{100,-80}}) {
      auto light=goodSensors();measured.beacon[1].signal=xy.first<200?5:50;
      for(uint8_t i=0;i<3;++i) {
        auto& b=measured.beacon[i];float d=hypotf(xy.first-b.x,xy.second-b.y);
        light.light[i]=lroundf(b.ambient+b.signal*powf(b.distance/d,b.exponent));
      }
      core::Localizer l;auto fix=l.update(light,measured);
      assert(fix.valid&&hypotf(fix.x-xy.first,fix.y-xy.second)<15);
    }
    measured.beacon[2]=measured.beacon[0];assert(!core::beaconGeometryValid(measured));
    auto missing=p;missing.beaconPositionMask=3;assert(!core::profileValid(missing));
  }
  auto s=goodSensors();auto pos=referencePose(p,core::CENTER);
  {
    core::Controller c(capture,true,true,p);assert(c.arm());assert(c.move(80,0,0,500,0));
    uint32_t t=0;advance(c,s,pos,t,500);assert(c.mode()==core::Mode::IDLE&&output[0]==0);
    assert(c.move(80,0,0,500,t));c.tick(t+251,s,pos);assert(c.mode()==core::Mode::FAULT&&output[0]==0);
    assert(!c.arm());c.stop();assert(c.arm());assert(!c.move(std::numeric_limits<float>::quiet_NaN(),0,0,500,t));
    c.stop();assert(c.arm());t=0xffffff00u;assert(c.move(60,0,0,500,t));advance(c,s,pos,t,520);assert(c.mode()==core::Mode::IDLE);
  }
  {
    auto incomplete=p;incomplete.point[core::BOX_RIGHT].samples=1;
    core::Controller c(capture,true,true,incomplete);assert(c.arm());assert(!c.start(0,s,pos));assert(!strcmp(c.fault(),"CALIBRATION_REQUIRED"));
    c.stop();assert(c.arm());incomplete=p;assert(c.start(0,s,pos));auto bad=s;bad.range[1]=-1;c.tick(20,bad,pos);assert(c.mode()==core::Mode::FAULT);
    c.stop();assert(c.arm());assert(c.start(0,s,pos));auto invalid=pos;invalid.valid=false;c.tick(20,s,invalid);assert(c.mode()==core::Mode::FAULT);
  }
  {
    core::Controller c(capture,true,true,p);assert(c.arm());uint32_t t=0;assert(c.start(0,s,pos));advance(c,s,pos,t,740);
    auto dark=s;dark.rgba[0]=dark.rgba[1]=dark.rgba[2]=100;advance(c,dark,pos,t,5200);assert(c.mode()==core::Mode::FAULT&&!strcmp(c.fault(),"COLOR_UNKNOWN"));
    c.stop();assert(c.arm());assert(c.demo(0));t=0;advance(c,s,pos,t,4300);assert(c.mode()==core::Mode::IDLE);
  }
  {
    auto optical=s;float trueX=650,trueY=120;
    for(uint8_t i=0;i<3;++i) { float d=hypotf(trueX-p.beacon[i].x,trueY-p.beacon[i].y);optical.light[i]=lroundf(10+50*powf(1000/d,2)); }
    core::Localizer l;auto fit=l.update(optical,p);assert(fit.valid&&!fit.headingValid);assert(hypotf(fit.x-trueX,fit.y-trueY)<10);
    l.resetHeading(180,0);fit=l.update(optical,p);assert(fit.headingValid&&fabsf(core::wrap(fit.yaw-180))<0.01f);
    optical.light[0]=1023;assert(!l.update(optical,p).valid);
    for(uint8_t i=0;i<3;++i) { float d=hypotf(300-p.beacon[i].x,-p.beacon[i].y);optical.light[i]=lroundf(10+50*powf(1000/d,2)); }
    fit=l.update(optical,p);assert(fit.valid&&fit.ultrasonic&&fit.reference==core::CENTER);assert(hypotf(fit.x-300,fit.y)<5);
    optical.range[1]=-1;assert(!l.update(optical,p).ultrasonic);
  }
  {
    auto taught=p;assert(core::captureReference(taught,core::CENTER,s,300,0,180));assert(taught.point[0].samples==1);
    auto shifted=s;shifted.range[0]+=35;shifted.range[1]+=35;
    assert(core::captureDerivative(taught,0,0,shifted,50));assert(fabsf(taught.point[0].jacobian[0][0]-.7f)<.001f);
    assert(!core::captureDerivative(taught,0,2,shifted,50));assert(!core::captureReference(taught,0,s,-1,0,0));
    assert(core::captureColor(taught,core::Color::GREEN,s));assert(core::classify(s,taught)==core::Color::GREEN);
  }
  {
    assert(profileSave(p));core::Profile loaded;assert(profileLoad(loaded));assert(core::profileValid(loaded));
    auto changed=p;changed.point[core::BOX_LEFT].y=-600;
    EEPROM.writesUntilFailure=40;try { profileSave(changed);assert(false); }catch(const std::runtime_error&) {}
    EEPROM.writesUntilFailure=-1;assert(profileLoad(loaded));assert(loaded.point[core::BOX_LEFT].y==-400);
    assert(profileSave(changed));assert(profileLoad(loaded)&&loaded.point[core::BOX_LEFT].y==-600);
  }
  mockSensors=goodSensors();setup();assert(robot.mode()==core::Mode::DISARMED);
  send("SURVEY MOUNTS 90 70 90 -70");send("SURVEY BEGIN 600 0 180");
  assert(survey.session());send("ARM");assert(robot.mode()==core::Mode::DISARMED);
  send("SURVEY TAKE WALL");assert(survey.busy());send("FWD 500");assert(robot.mode()==core::Mode::DISARMED);
  send("SURVEY STATUS");assert(Serial.output.str().find("# SURVEY {")!=std::string::npos);
  send("STOP");assert(!survey.session()&&!survey.busy());

  send("HEADING 180");send("TEACH CENTER 300 0 180");assert(profile.point[0].samples==1);
  send("TEACH CENTER nan 0 180");assert(profile.point[0].samples==1);
  send("BEACON 0 180 750 0 10 50 1000 2");assert(profile.beacon[0].signal==50);
  assert(profile.beaconPositionMask&1);
  send("CALIB");assert(Serial.output.str().find("# PROFILE {")!=std::string::npos);
  send("FWD 500");assert(!robot.armed());send("STOP");send("ARM");
#if MOTOR_WIRING_CONFIRMED
  assert(robot.mode()==core::Mode::IDLE);send("FWD 500");assert(robot.mode()==core::Mode::MANUAL);
  for(uint8_t i=0;i<4;++i) { assert(pwmValues[MOTOR_PWM[i]]>0);assert(directionValues[MOTOR_IN1[i]]==LOW); }
  for(int i=0;i<26;++i) { fakeMs+=20;loop(); }assert(robot.mode()==core::Mode::IDLE);
  send("STOP");send("ARM");send("FWD 500");send(std::string(160,'X')+"ARM");assert(robot.mode()==core::Mode::FAULT);
  send("STOP");send("ARM");send("DRIVE 80 0 nan 500");assert(robot.mode()==core::Mode::FAULT);
  send("STOP");send("ARM");send("FWD 500");send("!");assert(!robot.armed());
  send("STOP");send("ARM");Serial.input.push_back('A');loop();fakeMs+=60;loop();assert(robot.mode()==core::Mode::MANUAL);
  send("STOP");send("ARM");send("DEMO");assert(robot.mode()==core::Mode::DEMO);send("STOP");
#if BLUETOOTH_UART != 0
#if BLUETOOTH_UART == 2
  Stream& bt=Serial2;
#else
  Stream& bt=Serial3;
#endif
  send("ARM");send("FWD 500",bt);assert(robot.mode()==core::Mode::IDLE);send("STOP",bt);assert(!robot.armed());
  send("ARM",bt);send("FWD 500",bt);assert(robot.mode()==core::Mode::MANUAL);send("STOP");
#endif
#else
  assert(robot.mode()==core::Mode::FAULT);assert(pinWrites==0);
#endif
  telemetrySent=telemetrySize;queueTelemetry(fakeMs,mockSensors);std::string row(telemetryRow,telemetrySize);
  assert(std::count(row.begin(),row.end(),',')==34&&row.back()=='\n');
  assert(MOTOR_PWM[0]==12&&MOTOR_PWM[1]==8&&MOTOR_PWM[2]==9&&MOTOR_PWM[3]==5);
#if MOEBIUS_LEGACY_M4_PINS
  assert(MOTOR_IN1[3]==26&&MOTOR_IN2[3]==27);
#else
  assert(MOTOR_IN1[3]==A4&&MOTOR_IN2[3]==A5);
#endif
  std::cout<<"Both routes, localization, calibration recovery, parser and timer checks passed\n";
}
