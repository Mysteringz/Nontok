#include "control.h"
#include <math.h>
namespace core {
static float clamp(float x,float limit) { return fmaxf(-limit,fminf(limit,x)); }
static bool autonomous(Mode m) { return m>=Mode::TO_CENTER&&m<=Mode::DOCK; }
static bool active(Mode m) { return m==Mode::MANUAL||m==Mode::DEMO||autonomous(m); }
Controller::Controller(Output o,bool m,bool s,Profile& p,const Settings& cfg):output_(o),motorsReady_(m),sensorsReady_(s),profile_(p),cfg_(cfg) {}
bool Controller::armed() const { return mode_!=Mode::DISARMED&&mode_!=Mode::FAULT&&mode_!=Mode::FINISHED; }
void Controller::drive(float x,float y,float z) { vx_=x;vy_=y;wz_=z;output_(x,y,z); }
void Controller::enter(Mode m,uint32_t now) { drive(0,0,0);mode_=m;entered_=now;stable_=false; }
void Controller::stop() { drive(0,0,0);mode_=Mode::DISARMED;fault_="NONE";target_=-1; }
void Controller::fail(const char* reason) { drive(0,0,0);mode_=Mode::FAULT;fault_=reason; }
bool Controller::arm() {
  if(!motorsReady_) { fail("MOTOR_WIRING_UNCONFIRMED");return false; }
  if(mode_!=Mode::DISARMED&&mode_!=Mode::IDLE) return false;
  mode_=Mode::IDLE;fault_="NONE";decision_=Color::UNKNOWN;drive(0,0,0);return true;
}
bool Controller::move(float x,float y,float z,uint32_t ms,uint32_t now) {
  if(mode_!=Mode::IDLE&&mode_!=Mode::MANUAL) return false;
  if(!isfinite(x)||!isfinite(y)||!isfinite(z)||hypotf(x,y)>150||fabsf(z)>45||ms<1||ms>2000) { fail("BAD_MOVE");return false; }
  enter(Mode::MANUAL,now);duration_=ms;lastTick_=now;drive(x,y,z);return true;
}
bool Controller::demo(uint32_t now) { if(mode_!=Mode::IDLE) return false;enter(Mode::DEMO,now);demoStep_=0;lastTick_=now;return true; }
bool Controller::start(uint32_t now,const Sensors& s,const Pose& p) {
  if(mode_!=Mode::IDLE) return false;
  if(!sensorsReady_||!profileValid(profile_)) { fail("CALIBRATION_REQUIRED");return false; }
  if(!s.imuValid||!s.colorValid||s.range[0]<0||s.range[1]<0||!p.valid||!p.headingValid) { fail("SENSORS_NOT_READY");return false; }
  target_=CENTER;decision_=Color::UNKNOWN;started_=lastTick_=now;enter(Mode::TO_CENTER,now);return true;
}
bool Controller::stableFor(bool c,uint32_t now,uint32_t ms) {
  if(!c) { stable_=false;return false; } if(!stable_) { stable_=true;stableSince_=now; } return now-stableSince_>=ms;
}
void Controller::navigate(const Pose& p,const Reference& r,bool fine) {
  float dx=r.x-p.x,dy=r.y-p.y,rad=p.yaw*0.01745329252f;
  float error=wrap(r.yaw-p.yaw),speed=fine?25:80;
  float x=clamp((dx*cosf(rad)+dy*sinf(rad))*0.65f,speed);
  float y=clamp((-dx*sinf(rad)+dy*cosf(rad))*0.65f,speed);
  if(fabsf(error)>15) x=y=0;
  if(hypotf(dx,dy)<profile_.positionTolerance*0.6f) x=y=0;
  drive(x,y,fabsf(error)<profile_.yawTolerance*0.5f?0:clamp(error*1.5f,fine?12:25));
}
bool Controller::atReference(const Sensors& s,const Pose& p,uint8_t id) const {
  const Reference& r=profile_.point[id];
  return p.ultrasonic&&p.reference==id&&hypotf(p.x-r.x,p.y-r.y)<=profile_.positionTolerance&&
    fabsf(wrap(p.yaw-r.yaw))<=profile_.yawTolerance&&
    fabsf(s.range[0]-r.range[0])<=profile_.rangeTolerance&&fabsf(s.range[1]-r.range[1])<=profile_.rangeTolerance;
}
void Controller::tick(uint32_t now,const Sensors& s,const Pose& p) {
  if(!active(mode_)) return;
  if(now-lastTick_>250) { fail("LOOP_STALLED");return; } lastTick_=now;
  if(sensorsReady_) {
    // Only two front diagonal sensors exist; rear/side motion needs a clear test area.
    if((vx_>0||autonomous(mode_))&&(s.range[0]<0||s.range[1]<0)) { fail("RANGE_LOST");return; }
    if((vx_>0||autonomous(mode_))&&(s.range[0]<cfg_.manualGuard||s.range[1]<cfg_.manualGuard)) { fail("CLEARANCE");return; }
  }
  if(mode_==Mode::MANUAL) { if(now-entered_>=duration_) enter(Mode::IDLE,now);return; }
  if(mode_==Mode::DEMO) {
    static const int8_t seq[6][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    if(now-entered_<400) drive(seq[demoStep_][0]*60,seq[demoStep_][1]*60,seq[demoStep_][2]*20);else drive(0,0,0);
    if(now-entered_>=700) { if(++demoStep_==6) enter(Mode::IDLE,now);else entered_=now; } return;
  }
  if(!s.imuValid||!p.valid||!p.headingValid) { fail("LOCALIZATION_LOST");return; }
  if(s.range[0]<profile_.guard||s.range[1]<profile_.guard) { fail("CLEARANCE");return; }
  if(now-started_>120000||now-entered_>40000) { fail("MISSION_TIMEOUT");return; }
  if(mode_==Mode::TO_CENTER) {
    navigate(p,profile_.point[CENTER],false);
    if(stableFor(atReference(s,p,CENTER),now,700)) { enter(Mode::READ_COLOR,now);votes_=0;vote_=Color::UNKNOWN;lastColorSequence_=s.colorSequence; }
  } else if(mode_==Mode::READ_COLOR) {
    drive(0,0,0);
    if(!atReference(s,p,CENTER)) { fail("CENTER_MOVED");return; }
    if(now-entered_>5000) { fail("COLOR_UNKNOWN");return; }
    if(s.colorSequence!=lastColorSequence_) {
      lastColorSequence_=s.colorSequence;Color c=classify(s,profile_);
      if(c==Color::UNKNOWN) { votes_=0;vote_=c; }
      else if(c==vote_) ++votes_;else { vote_=c;votes_=1; }
      if(votes_>=5) {
        decision_=c;target_=c==Color::GREEN?TURN_LEFT:TURN_RIGHT;
        turnGyro_=s.heading+(c==Color::GREEN?90:-90);enter(Mode::TURN,now);
      }
    }
  } else if(mode_==Mode::TURN) {
    const Reference& r=profile_.point[target_];
    float gyroError=wrap(turnGyro_-s.heading),fusedError=wrap(r.yaw-p.yaw);
    if(hypotf(p.x-r.x,p.y-r.y)>profile_.localRadius) { fail("TURN_POSITION_LOST");return; }
    drive(0,0,fabsf(gyroError)<0.6f&&fabsf(fusedError)<0.6f?0:clamp(0.7f*gyroError+0.8f*fusedError,22));
    if(stableFor(fabsf(gyroError)<=profile_.yawTolerance&&atReference(s,p,target_),now,600)) {
      target_=decision_==Color::GREEN?BOX_LEFT:BOX_RIGHT;enter(Mode::TO_BOX,now);
    }
  } else if(mode_==Mode::TO_BOX||mode_==Mode::DOCK) {
    const Reference& r=profile_.point[target_];
    if(mode_==Mode::TO_BOX&&hypotf(p.x-r.x,p.y-r.y)<profile_.localRadius*0.6f) enter(Mode::DOCK,now);
    navigate(p,r,mode_==Mode::DOCK);
    if(stableFor(atReference(s,p,target_),now,1000)) enter(Mode::FINISHED,now);
  }
}
const char* modeName(Mode m) {
  switch(m) {
    case Mode::DISARMED:return "DISARMED";case Mode::IDLE:return "IDLE";case Mode::MANUAL:return "MANUAL";
    case Mode::DEMO:return "DEMO";case Mode::TO_CENTER:return "TO_CENTER";case Mode::READ_COLOR:return "READ_COLOR";
    case Mode::TURN:return "TURN";case Mode::TO_BOX:return "TO_BOX";case Mode::DOCK:return "DOCK";
    case Mode::FINISHED:return "FINISHED";default:return "FAULT";
  }
}
}
