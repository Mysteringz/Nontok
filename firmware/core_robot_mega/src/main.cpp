#include "config.h"
#include "control.h"
#include "sensors.h"
#include "display.h"
#include "calibration.h"
#include "survey.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static void motorOutput(float vx, float vy, float wz) {
#if MOTOR_WIRING_CONFIRMED
  float rotation = wz * DEG_TO_RAD * ROTATION_ARM_MM;
  float wheel[4] = {vx-vy-rotation, vx+vy+rotation, vx+vy-rotation, vx-vy+rotation};
  float peak = MAX_WHEEL_MM_S;
  for (uint8_t i=0; i<4; ++i) if (fabs(wheel[i]) > peak) peak = fabs(wheel[i]);
  for (uint8_t i=0; i<4; ++i) {
    float speed = wheel[i] * MOTOR_SIGN[i] * MAX_WHEEL_MM_S / peak;
    uint8_t pwm = fabs(speed)<1 ? 0 : MIN_PWM + (uint8_t)((MAX_PWM-MIN_PWM)*fabs(speed)/MAX_WHEEL_MM_S);
    analogWrite(MOTOR_PWM[i],0);
    digitalWrite(MOTOR_IN1[i],speed>=0?HIGH:LOW);
    digitalWrite(MOTOR_IN2[i],speed>=0?LOW:HIGH);
    analogWrite(MOTOR_PWM[i],pwm);
  }
#else
  (void)vx; (void)vy; (void)wz;
#endif
}
static core::Profile profile;
static core::Localizer localizer;
static core::Pose pose;
static core::Survey survey;
static uint8_t surveyOwner=255;
static core::Controller robot(motorOutput,MOTOR_WIRING_CONFIRMED,SENSOR_WIRING_CONFIRMED,profile);
struct CommandPort {
  Stream* stream;
  char line[144];
  uint8_t used = 0;
  bool dropping = false;
  uint32_t lastByte = 0;
  explicit CommandPort(Stream& s) : stream(&s) {}
};
static CommandPort usb(Serial);
#if BLUETOOTH_UART == 2
static CommandPort bluetooth(Serial2);
#elif BLUETOOTH_UART == 3
static CommandPort bluetooth(Serial3);
#endif
static uint8_t owner = 255;
static float manualSpeed = 80;
static uint32_t lastTelemetry = 0;
static char telemetryRow[480];
static uint16_t telemetrySize=0, telemetrySent=0;

static bool number(char* token, float& value) {
  if (!token) return false;
  char* end;
  value = strtod(token,&end);
  return end!=token && !*end && isfinite(value);
}
static bool duration(char* token, uint32_t& value) {
  if (!token) { value=500; return true; }
  for (char* p=token; *p; ++p) if (*p<'0'||*p>'9') return false;
  char* end; unsigned long result=strtoul(token,&end,10);
  if (*end || result<1 || result>2000) return false;
  value=result; return true;
}
static void reply(CommandPort& p, const char* text) {
  // Do not splice command replies into a partially transmitted CSV record.
  if (p.stream==&Serial && telemetrySent<telemetrySize) {
    if (telemetrySent) Serial.println();
    telemetrySize=telemetrySent=0;
  }
  p.stream->print(F("# ")); p.stream->println(text);
}
static void bad(CommandPort& p) { robot.fail("BAD_COMMAND"); reply(p,"ERR BAD_COMMAND"); }
static void header(Stream& out) {
  out.println(F("ms,mode,armed,left45_mm,right45_mm,gyro_deg,gyro_dps,imu_ok,light_left,light_middle,light_right,light_ok,red,green,blue,clear,color_ok,color,decision,x_mm,y_mm,yaw_deg,pose_ok,heading_ok,beacon_residual_mm,us_fused,us_residual_mm,reference,target,vx,vy,wz,calibrated,hardware,fault"));
}
static void decimal(char* target,size_t size,float value) {
  if(!isfinite(value)||fabsf(value)>1000000) { strncpy(target,"0",size);return; }
  long scaled=lroundf(value*10);unsigned long magnitude=scaled<0?(unsigned long)(-scaled):(unsigned long)scaled;
  snprintf(target,size,"%s%lu.%lu",scaled<0?"-":"",magnitude/10,magnitude%10);
}
static void queueTelemetry(uint32_t now,const core::Sensors& s) {
  if(telemetrySent<telemetrySize) return;
  char value[10][16];float data[]={s.heading,s.gyroRate,pose.x,pose.y,pose.yaw,pose.residual,pose.rangeError,robot.vx(),robot.vy(),robot.wz()};
  for(uint8_t i=0;i<10;++i) decimal(value[i],sizeof(value[i]),data[i]);
  unsigned hardware=(MOTOR_WIRING_CONFIRMED?1:0)|(SENSOR_WIRING_CONFIRMED?2:0)|(LIGHT_DRIVER?4:0)|(COLOR_DRIVER?8:0);
  int n=snprintf(telemetryRow,sizeof(telemetryRow),
    "%lu,%s,%u,%d,%d,%s,%s,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%s,%s,%s,%s,%s,%u,%u,%s,%u,%s,%s,%s,%s,%s,%s,%u,%u,%s\n",
    (unsigned long)now,core::modeName(robot.mode()),robot.armed()?1:0,s.range[0],s.range[1],value[0],value[1],s.imuValid?1:0,
    s.light[0],s.light[1],s.light[2],s.lightValid?1:0,s.rgba[0],s.rgba[1],s.rgba[2],s.rgba[3],s.colorValid?1:0,
    core::colorName(core::classify(s,profile)),core::colorName(robot.decision()),value[2],value[3],value[4],pose.valid?1:0,pose.headingValid?1:0,
    value[5],pose.ultrasonic?1:0,value[6],pose.reference<0?"NONE":core::pointName(pose.reference),robot.target()<0?"NONE":core::pointName(robot.target()),
    value[7],value[8],value[9],core::profileValid(profile)?1:0,hardware,robot.fault());
  telemetrySize=n>0&&n<(int)sizeof(telemetryRow)?n:0;telemetrySent=0;
}
static int pointId(const char* token) {
  if(token) for(uint8_t i=0;i<core::POINTS;++i) if(!strcmp(token,core::pointName(i))) return i;
  return -1;
}
static bool stopped() { return robot.mode()==core::Mode::DISARMED; }
static void dumpWalls(Stream& out,const core::SurveyCalibration& cfg) {
  out.print(F("{\"valid\":"));out.print(cfg.wallsValid?1:0);out.print(F(",\"width_mm\":"));out.print(cfg.measuredWidth,2);
  out.print(F(",\"walls\":["));
  for(uint8_t i=0;i<3;++i) {
    const core::SurveyWall& w=cfg.wall[i];if(i) out.print(',');
    out.print(F("{\"ok\":"));out.print(w.valid?1:0);out.print(F(",\"intercept_mm\":"));out.print(w.intercept,2);
    out.print(F(",\"slope\":"));out.print(w.slope,5);out.print(F(",\"rms_mm\":"));out.print(w.rms,2);
    out.print(F(",\"from_mm\":"));out.print(w.alongMin,2);out.print(F(",\"to_mm\":"));out.print(w.alongMax,2);
    out.print(F(",\"points\":"));out.print((unsigned)w.points);out.print('}');
  }
  out.print(F("]}"));
}
static void dumpSurvey(CommandPort& p,bool solving=false) {
  reply(p,"SURVEY_STATUS");Stream& out=*p.stream;
  out.print(F("# SURVEY {\"phase\":\""));out.print(solving?"SOLVING":survey.phase());
  out.print(F("\",\"active\":"));out.print(survey.session()?1:0);out.print(F(",\"count\":"));out.print((unsigned)survey.count());
  out.print(F(",\"capacity\":"));out.print((unsigned)core::SURVEY_CAPACITY);out.print(F(",\"message\":\""));out.print(survey.message());out.print('"');
  if(survey.busy()||solving) { out.println('}');return; }
  const core::SurveyCalibration& cfg=survey.calibration();
  out.print(F(",\"mounts_known\":"));out.print(cfg.mountsKnown?1:0);out.print(F(",\"ambient_known\":"));out.print(cfg.ambientKnown?1:0);
  out.print(F(",\"optics_known\":"));out.print(cfg.opticsKnown?1:0);out.print(F(",\"pose\":["));out.print(survey.poseX(),1);out.print(',');out.print(survey.poseY(),1);out.print(']');
  out.print(F(",\"mounts\":["));
  for(uint8_t i=0;i<2;++i) { if(i) out.print(',');out.print('[');out.print(cfg.mount[i].forward,1);out.print(',');out.print(cfg.mount[i].left,1);out.print(']'); }
  out.print(F("],\"ambient\":["));for(uint8_t i=0;i<3;++i) { if(i) out.print(',');out.print(cfg.ambient[i],2); }
  out.print(F("],\"height\":["));for(uint8_t i=0;i<3;++i) { if(i) out.print(',');out.print(cfg.height[i],1); }
  out.print(F("],\"exponent\":"));out.print(cfg.exponent,3);out.print(F(",\"map\":"));dumpWalls(out,cfg);
  out.print(F(",\"beacons\":["));
  for(uint8_t i=0;i<3;++i) {
    const core::BeaconFit& f=survey.beacon(i);if(i) out.print(',');out.print(F("{\"ok\":"));out.print(f.valid?1:0);
    out.print(F(",\"x_mm\":"));out.print(f.model.x,1);out.print(F(",\"y_mm\":"));out.print(f.model.y,1);
    out.print(F(",\"rms_mm\":"));out.print(f.rms,2);out.print(F(",\"sensitivity_mm\":"));out.print(f.sensitivity,2);
    out.print(F(",\"ref_signal\":"));out.print(f.model.signal,2);out.print(F(",\"points\":"));out.print((unsigned)f.points);out.print('}');
  }
  out.println(F("]}"));
}
static void dumpSurveySample(CommandPort& p,uint8_t index) {
  reply(p,"SURVEY_SAMPLE");Stream& out=*p.stream;const core::SurveySample& s=survey.sample(index);
  out.print(F("# SURVEY_SAMPLE {\"index\":"));out.print((unsigned)index);out.print(F(",\"ms\":"));out.print((unsigned long)s.ms);
  out.print(F(",\"x_mm\":"));out.print(s.x,1);out.print(F(",\"y_mm\":"));out.print(s.y,1);out.print(F(",\"yaw_deg\":"));out.print(s.yaw,2);
  out.print(F(",\"optical\":"));out.print(s.optical?1:0);out.print(F(",\"ranges\":["));out.print(s.range[0]);out.print(',');out.print(s.range[1]);
  out.print(F("],\"lights\":["));for(uint8_t i=0;i<3;++i) { if(i) out.print(',');out.print((unsigned)s.light[i]); }out.println(F("]}"));
}
static bool values(float* data,uint8_t count) {
  bool valid=true;for(uint8_t i=0;i<count;++i) valid=number(strtok(NULL," \t"),data[i])&&valid;
  return !strtok(NULL," \t")&&valid;
}
static void surveyCommand(CommandPort& p,uint8_t id) {
  char* op=strtok(NULL," \t");if(!op) op=(char*)"STATUS";
  if(!stopped()) { reply(p,"ERR STOP_BEFORE_SURVEY");return; }
  bool stop=!strcmp(op,"END")||!strcmp(op,"CANCEL");
  bool status=!strcmp(op,"STATUS");
  if(surveyOwner!=255&&surveyOwner!=id&&!stop&&!status) { reply(p,"ERR OTHER_SURVEY_CONTROLLER");return; }
  if(survey.busy()&&!stop&&!status) { reply(p,"ERR SURVEY_BUSY");return; }
  bool ok=false;float data[4]={};
  if(status) { if(strtok(NULL," \t")) bad(p);else dumpSurvey(p);return; }
  if(stop) {
    if(!strtok(NULL," \t")) { if(!strcmp(op,"END")) { survey.end();surveyOwner=255; }else survey.cancel();ok=true; }
  } else if(!strcmp(op,"MOUNTS")) { if(values(data,4)) ok=survey.mounts(data[0],data[1],data[2],data[3]); }
  else if(!strcmp(op,"OPTICS")) { if(values(data,4)) ok=survey.optics(data[0],data[1],data[2],data[3]); }
  else if(!strcmp(op,"BEGIN")) {
    if(values(data,3)) { ok=survey.begin(data[0],data[1],data[2],sensorsRead());if(ok) surveyOwner=id; }
  } else if(!strcmp(op,"POSE")) { if(values(data,2)) ok=survey.setPose(data[0],data[1]); }
  else if(!strcmp(op,"TAKE")||!strcmp(op,"DARK")) {
    core::CaptureKind kind=core::CaptureKind::DARK;
    if(!strcmp(op,"TAKE")) {
      char* name=strtok(NULL," \t");kind=name&&!strcmp(name,"WALL")?core::CaptureKind::WALL:name&&!strcmp(name,"POINT")?core::CaptureKind::POINT:core::CaptureKind::NONE;
    }
    if(!strtok(NULL," \t")&&kind!=core::CaptureKind::NONE) { ok=survey.capture(kind,millis(),sensorsRead());if(ok) surveyOwner=id; }
  } else if(!strcmp(op,"SOLVE")) {
    if(!strtok(NULL," \t")) { dumpSurvey(p,true);ok=survey.solve(); }
  } else if(!strcmp(op,"APPLY")) {
    if(!strtok(NULL," \t")) { ok=survey.apply(profile);if(ok) { localizer.invalidate();surveyOwner=255; } }
  } else if(!strcmp(op,"CLEAR")) {
    if(!strtok(NULL," \t")) { survey.clear();surveyOwner=255;ok=true; }
  } else if(!strcmp(op,"DUMP")) {
    if(!strtok(NULL," \t")) { for(uint8_t i=0;i<survey.count();++i) dumpSurveySample(p,i);ok=true; }
  }
  reply(p,ok?"OK SURVEY":"ERR SURVEY");dumpSurvey(p);
}
static void dumpProfile(CommandPort& p) {
  reply(p,"CALIBRATION");Stream& out=*p.stream;
  out.print(F("# PROFILE {\"valid\":"));out.print(core::profileValid(profile)?1:0);
  out.print(F(",\"field_width_mm\":"));out.print(core::FIELD_WIDTH_MM,1);
  out.print(F(",\"beacon_position_mask\":"));out.print((unsigned)profile.beaconPositionMask);
  out.print(F(",\"beacons\":["));
  for(uint8_t i=0;i<3;++i) {
    if(i) out.print(',');
    const core::Beacon& b=profile.beacon[i];
    out.print('[');out.print(b.x,3);out.print(',');out.print(b.y,3);out.print(',');out.print(b.height,3);out.print(',');out.print(b.ambient,3);
    out.print(',');out.print(b.signal,3);out.print(',');out.print(b.distance,3);out.print(',');out.print(b.exponent,3);out.print(']');
  }
  out.print(F("],\"survey_map\":"));dumpWalls(out,profile.survey);
  out.print(F(",\"points\":["));
  for(uint8_t i=0;i<core::POINTS;++i) {
    const core::Reference& r=profile.point[i];if(i) out.print(',');
    out.print(F("{\"name\":\""));out.print(core::pointName(i));out.print(F("\",\"samples\":"));out.print((unsigned)r.samples);
    out.print(F(",\"x\":"));out.print(r.x,2);out.print(F(",\"y\":"));out.print(r.y,2);out.print(F(",\"yaw\":"));out.print(r.yaw,2);
    out.print(F(",\"ranges\":["));out.print(r.range[0],2);out.print(',');out.print(r.range[1],2);out.print(F("],\"jacobian\":["));
    for(uint8_t j=0;j<2;++j) { if(j) out.print(',');out.print('[');for(uint8_t k=0;k<3;++k) { if(k) out.print(',');out.print(r.jacobian[j][k],4); }out.print(']'); }
    out.print(F("],\"lights\":["));
    for(uint8_t j=0;j<3;++j) { if(j) out.print(',');out.print((unsigned)r.light[j]); }
    out.print(F("],\"captured_gyro\":"));out.print(r.capturedGyro,3);out.print('}');
  }
  out.print(F("],\"color_mask\":"));out.print((unsigned)profile.colorMask);out.print(F(",\"colors\":["));
  for(uint8_t i=0;i<2;++i) { if(i) out.print(',');out.print('[');for(uint8_t j=0;j<3;++j) { if(j) out.print(',');out.print(profile.color[i][j],4); }out.print(']'); }
  out.println(F("]}"));
}
static bool calibration(CommandPort& p,const char* op) {
  bool known=!strcmp(op,"CALIB")||!strcmp(op,"PROBE")||!strcmp(op,"HEADING")||!strcmp(op,"TEACH")||!strcmp(op,"SAMPLE")||
    !strcmp(op,"COLOR")||!strcmp(op,"BEACON")||!strcmp(op,"SAVE")||!strcmp(op,"LOAD")||!strcmp(op,"DEFAULTS");
  if(!known) return false;
  if(!stopped()) { reply(p,"ERR STOP_BEFORE_CALIBRATION");return true; }
  core::Sensors s=sensorsRead();bool ok=false;
  if(!strcmp(op,"CALIB")) { if(strtok(NULL," \t")) bad(p);else dumpProfile(p);return true; }
  if(!strcmp(op,"PROBE")) { if(strtok(NULL," \t")) bad(p);else { reply(p,"PROBING");sensorsProbe(*p.stream); }return true; }
  if(!strcmp(op,"HEADING")) {
    float degrees;if(number(strtok(NULL," \t"),degrees)&&!strtok(NULL," \t")&&fabsf(degrees)<=360&&s.imuValid) {
      localizer.resetHeading(degrees,s.heading);ok=true;
    }
  } else if(!strcmp(op,"SAVE")) {
    if(!strtok(NULL," \t")) ok=profileSave(profile);
  } else if(!strcmp(op,"LOAD")) {
    if(!strtok(NULL," \t")) { ok=profileLoad(profile);localizer.invalidate();if(ok) { survey.clear();survey.configure(profile.survey);surveyOwner=255; } }
  } else if(!strcmp(op,"DEFAULTS")) {
    if(!strtok(NULL," \t")) { core::defaultProfile(profile);localizer.invalidate();survey.clear();survey.configure(profile.survey);surveyOwner=255;ok=true; }
  } else if(!strcmp(op,"COLOR")) {
    char* name=strtok(NULL," \t");core::Color c=name&&!strcmp(name,"RED")?core::Color::RED:name&&!strcmp(name,"GREEN")?core::Color::GREEN:core::Color::UNKNOWN;
    if(!strtok(NULL," \t")) ok=core::captureColor(profile,c,s);
  } else if(!strcmp(op,"TEACH")) {
    int id=pointId(strtok(NULL," \t"));float x=0,y=0,yaw=0;
    bool valid=number(strtok(NULL," \t"),x);valid=number(strtok(NULL," \t"),y)&&valid;valid=number(strtok(NULL," \t"),yaw)&&valid;
    if(valid&&!strtok(NULL," \t")&&id>=0) ok=core::captureReference(profile,id,s,x,y,yaw);
  } else if(!strcmp(op,"SAMPLE")) {
    int id=pointId(strtok(NULL," \t"));char* axis=strtok(NULL," \t");float offset=0;
    int a=axis&&!strcmp(axis,"X")?0:axis&&!strcmp(axis,"Y")?1:axis&&!strcmp(axis,"YAW")?2:-1;
    if(number(strtok(NULL," \t"),offset)&&!strtok(NULL," \t")&&id>=0&&a>=0) ok=core::captureDerivative(profile,id,a,s,offset);
  } else if(!strcmp(op,"BEACON")) {
    float values[8]={};bool valid=true;
    for(uint8_t i=0;i<8;++i) valid=number(strtok(NULL," \t"),values[i])&&valid;
    int id=valid&&values[0]>=0&&values[0]<=2?(int)values[0]:-1;
    if(valid&&!strtok(NULL," \t")&&values[0]==id&&id>=0&&id<3) {
      core::Beacon b;b.x=values[1];b.y=values[2];b.height=values[3];b.ambient=values[4];b.signal=values[5];b.distance=values[6];b.exponent=values[7];
      if(fabsf(b.x)<=10000&&fabsf(b.y)<=10000&&b.height>=0&&b.height<=3000&&b.ambient>=0&&b.ambient<1000&&b.signal>=5&&b.signal<=1023&&b.distance>=50&&b.distance<=5000&&b.exponent>=0.5f&&b.exponent<=5) { profile.beacon[id]=b;profile.beaconPositionMask|=1<<id;ok=true; }
    }
  }
  reply(p,ok?"OK CALIBRATION":"ERR CALIBRATION");
  if(ok&&(!strcmp(op,"LOAD")||!strcmp(op,"DEFAULTS"))) dumpSurvey(p);
  return true;
}
static void pumpTelemetry() {
  int room=Serial.availableForWrite();
  uint16_t remaining=telemetrySize-telemetrySent;
  uint16_t count=room<(int)remaining?room:remaining;
  if (count) telemetrySent+=Serial.write((const uint8_t*)telemetryRow+telemetrySent,count);
}
static void command(CommandPort& port, uint8_t id) {
  char* op=strtok(port.line," \t");
  if (!op) return;
  if (!strcmp(op,"STOP")||!strcmp(op,"Z")) {
    if (strtok(NULL," \t")) { bad(port); return; }
    bool hadSurvey=survey.session()||survey.busy();
    robot.stop();survey.end();surveyOwner=255; owner=255; reply(port,"OK STOP");if(hadSurvey) dumpSurvey(port);return;
  }
  if (!strcmp(op,"STATUS")) {
    if (id==0) queueTelemetry(millis(),sensorsRead());
    else { reply(port,core::modeName(robot.mode())); reply(port,robot.fault()); }
    return;
  }
  if (!strcmp(op,"PING")) { reply(port,"PONG MEGA3"); return; }
  if (!strcmp(op,"HELP")) {
    reply(port,"ARM | STOP | START | DEMO | SPEED 1..150 | FWD/BACK/LEFT/RIGHT/CW/CCW [1..2000 ms]");
    reply(port,"DRIVE vx vy wz duration_ms | STATUS | PING | ! emergency stop"); return;
  }
  if(!strcmp(op,"SURVEY")) { surveyCommand(port,id);return; }
  if(survey.session()||survey.busy()) { reply(port,"ERR END_SURVEY_FIRST");return; }
  if(calibration(port,op)) return;
  if (owner!=255 && owner!=id) { reply(port,"ERR OTHER_CONTROLLER"); return; }
  if (!strcmp(op,"ARM")) {
    if (strtok(NULL," \t")) { bad(port); return; }
    if (robot.arm()) { owner=id; reply(port,"OK ARM"); }
    else { reply(port,"ERR ARM"); reply(port,robot.fault()); }
    return;
  }
  if (!strcmp(op,"SPEED")) {
    float speed;
    if (!number(strtok(NULL," \t"),speed)||strtok(NULL," \t")||speed<1||speed>150) { bad(port); return; }
    manualSpeed=speed; reply(port,"OK SPEED"); return;
  }
  if (!strcmp(op,"START")||!strcmp(op,"PARK")||!strcmp(op,"DEMO")) {
    if (strtok(NULL," \t")) { bad(port); return; }
    bool ok=!strcmp(op,"DEMO") ? robot.demo(millis()) : robot.start(millis(),sensorsRead(),pose);
    reply(port,ok?"OK STARTED":"ERR START"); return;
  }
  float x=0,y=0,z=0;
  uint32_t ms=500;
  if (!strcmp(op,"DRIVE")) {
    bool ok=number(strtok(NULL," \t"),x);
    ok=number(strtok(NULL," \t"),y)&&ok;
    ok=number(strtok(NULL," \t"),z)&&ok;
    char* timeToken=strtok(NULL," \t");
    if (!timeToken||!duration(timeToken,ms)||!ok||strtok(NULL," \t")) { bad(port); return; }
  } else {
    if (!strcmp(op,"FWD")||!strcmp(op,"A")) x=manualSpeed;
    else if (!strcmp(op,"BACK")||!strcmp(op,"E")) x=-manualSpeed;
    else if (!strcmp(op,"LEFT")||!strcmp(op,"G")) y=manualSpeed;
    else if (!strcmp(op,"RIGHT")||!strcmp(op,"C")) y=-manualSpeed;
    else if (!strcmp(op,"CW")) z=-25;
    else if (!strcmp(op,"CCW")) z=25;
    else if (!strcmp(op,"B")) { x=manualSpeed*0.7071f; y=-x; }
    else if (!strcmp(op,"D")) { x=-manualSpeed*0.7071f; y=x; }
    else if (!strcmp(op,"F")) { y=manualSpeed*0.7071f; x=-y; }
    else if (!strcmp(op,"H")) { x=manualSpeed*0.7071f; y=x; }
    else { bad(port); return; }
    if (!duration(strtok(NULL," \t"),ms)||strtok(NULL," \t")) { bad(port); return; }
  }
  reply(port,robot.move(x,y,z,ms,millis())?"OK MOVE":"ERR MOVE");
}
static void readPort(CommandPort& port, uint8_t id) {
  for (uint8_t count=0;count<48 && port.stream->available();++count) {
    char c=port.stream->read(); port.lastByte=millis();
    if (c=='!') { // Stop even in the middle of a damaged packet.
      bool hadSurvey=survey.session()||survey.busy();
      robot.stop();survey.end();surveyOwner=255; owner=255; port.used=0; port.dropping=true; reply(port,"OK STOP");if(hadSurvey) dumpSurvey(port);continue;
    }
    if (c=='\r') continue;
    if (c=='\n') {
      if (!port.dropping) { port.line[port.used]=0; command(port,id); }
      port.used=0; port.dropping=false;
      return; // One completed command per loop; flooding cannot starve motor timers.
    } else if (!port.dropping) {
      if (((uint8_t)c<32 && c!='\t') || (uint8_t)c>126 || port.used>=sizeof(port.line)-1) {
        robot.fail("BAD_LINE"); port.used=0; port.dropping=true; reply(port,"ERR BAD_LINE");
      } else port.line[port.used++]=c;
    }
  }
  // Vendor button apps may send a single A..H/Z without newline. Require ARM first.
  if (!port.dropping && port.used==1 && millis()-port.lastByte>=50 &&
      strchr("ABCDEFGHZ",port.line[0])) {
    port.line[1]=0; command(port,id); port.used=0;
  }
  // Interrupted text packets cannot be combined with future commands.
  if (port.used && millis()-port.lastByte>500) {
    port.used=0; port.dropping=true; robot.fail("PARTIAL_COMMAND");
  }
}
void setup() {
  Serial.begin(USB_BAUD);
#if BLUETOOTH_UART == 2
  Serial2.begin(BLUETOOTH_BAUD);
#elif BLUETOOTH_UART == 3
  Serial3.begin(BLUETOOTH_BAUD);
#endif
#if MOTOR_WIRING_CONFIRMED
  for (uint8_t i=0;i<4;++i) {
    pinMode(MOTOR_PWM[i],OUTPUT); analogWrite(MOTOR_PWM[i],0);
    pinMode(MOTOR_IN1[i],OUTPUT); pinMode(MOTOR_IN2[i],OUTPUT);
  }
#endif
  core::defaultProfile(profile);profileLoad(profile);survey.configure(profile.survey);
  robot.stop(); sensorsBegin(); displayBegin();
  Serial.println(F("# READY MEGA3")); header(Serial);
#if BLUETOOTH_UART != 0
  reply(bluetooth,"READY MEGA3");
#endif
}
void loop() {
  // Advance timers before accepting new motion so stale commands cannot mask a stall.
  core::Sensors s=sensorsRead(); robot.tick(millis(),s,pose);
  readPort(usb,0);
#if BLUETOOTH_UART != 0
  readPort(bluetooth,1);
#endif
  sensorsUpdate(); s=sensorsRead();
  uint8_t beforeSurvey=survey.count();
  if(!stopped()&&survey.busy()) survey.end();
  if(survey.update(millis(),s)) {
    CommandPort* destination=&usb;
#if BLUETOOTH_UART != 0
    if(surveyOwner==1) destination=&bluetooth;
#endif
    if(survey.count()>beforeSurvey) dumpSurveySample(*destination,survey.count()-1);
    dumpSurvey(*destination);
  }
  static uint32_t lastLocalization=0;
  if(millis()-lastLocalization>=50) { lastLocalization=millis();pose=localizer.update(s,profile); }
  robot.tick(millis(),s,pose);
  displayUpdate(robot,s);
  if (millis()-lastTelemetry>=100) {
    lastTelemetry=millis(); queueTelemetry(lastTelemetry,s);
  }
  pumpTelemetry();
}
