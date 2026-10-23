#include "survey.h"
#include <math.h>
namespace core {
static float clamp(float x,float n) { return fmaxf(-n,fminf(n,x)); }
static bool coordinates(float x,float y) { return isfinite(x)&&isfinite(y)&&x>0&&x<=4000&&fabsf(y)<FIELD_WIDTH_MM/2; }
static bool quiet(const Sensors& s) { return s.imuValid&&isfinite(s.heading)&&isfinite(s.gyroRate)&&fabsf(s.gyroRate)<1.5f; }
void Survey::invalidate() { solved_=false;cfg_.wallsValid=false;for(auto& f:fit_) f=BeaconFit(); }
bool Survey::mounts(float lx,float ly,float rx,float ry) {
  if(busy()||count_>0) { message_="CLEAR_CAPTURES_BEFORE_CHANGING_MOUNTS";return false; }
  const float values[]={lx,ly,rx,ry};
  for(float v:values) if(!isfinite(v)||fabsf(v)>500) return false;
  cfg_.mount[0].forward=lx;cfg_.mount[0].left=ly;cfg_.mount[1].forward=rx;cfg_.mount[1].left=ry;
  cfg_.mountsKnown=true;invalidate();message_="MOUNTS_SET";return true;
}
bool Survey::optics(float hl,float hm,float hr,float n) {
  if(busy()) return false;
  if(!isfinite(n)||n<0.5f||n>5) return false;
  float height[]={hl,hm,hr};for(float h:height) if(!isfinite(h)||h<0||h>3000) return false;
  for(uint8_t i=0;i<3;++i) cfg_.height[i]=height[i];
  cfg_.exponent=n;cfg_.opticsKnown=true;invalidate();message_="OPTICAL_ASSUMPTIONS_SET";return true;
}
bool Survey::begin(float x,float y,float yaw,const Sensors& s) {
  if(busy()||!coordinates(x,y)||!isfinite(yaw)||fabsf(yaw)>360||!quiet(s)||!cfg_.mountsKnown) { message_="NEED_MOUNTS_KNOWN_POSE_AND_IMU";return false; }
  count_=0;invalidate();x_=x;y_=y;offset_=wrap(yaw-s.heading);session_=true;message_="TURN_BY_HAND_THEN_CAPTURE";return true;
}
bool Survey::setPose(float x,float y) {
  if(!session_||busy()||!coordinates(x,y)) return false;
  x_=x;y_=y;message_="POSE_SET_KEEP_ROBOT_STILL";return true;
}
void Survey::cancel() { kind_=CaptureKind::NONE;solving_=false;message_="CAPTURE_CANCELLED"; }
void Survey::end() { cancel();session_=false;message_="SESSION_ENDED"; }
void Survey::clear() { end();count_=0;invalidate();message_="SAMPLES_CLEARED"; }
const char* Survey::phase() const {
  if(solving_) return "SOLVING";
  if(kind_==CaptureKind::DARK) return "DARK";
  if(kind_==CaptureKind::POINT) return "CAPTURE_POINT";
  if(kind_==CaptureKind::WALL) return "CAPTURE_WALL";
  return session_?"READY":"IDLE";
}
bool Survey::capture(CaptureKind k,uint32_t now,const Sensors& s) {
  if(busy()||k==CaptureKind::NONE||!quiet(s)||(k!=CaptureKind::DARK&&(!session_||count_>=SURVEY_CAPACITY))) { message_="CAPTURE_NOT_READY";return false; }
  if((k==CaptureKind::DARK||k==CaptureKind::POINT)&&!s.lightValid) { message_="LIGHT_INTERFACE_NOT_READY";return false; }
  kind_=k;started_=now;startHeading_=s.heading;headingSum_=0;headings_=0;lightSeen_=0;lastLight_=s.lightSequence;
  for(uint8_t i=0;i<3;++i) lightSum_[i]=0;
  for(uint8_t i=0;i<2;++i) {
    lastRange_[i]=s.rangeSequence[i];rangeSeen_[i]=rangeGood_[i]=0;rangeSum_[i]=0;low_[i]=32767;high_[i]=0;
  }
  message_=k==CaptureKind::DARK?"LIGHTS_MUST_BE_OFF":"SAMPLING_KEEP_STILL";return true;
}
bool Survey::update(uint32_t now,const Sensors& s) {
  if(kind_==CaptureKind::NONE) return false;
  if(!quiet(s)||fabsf(wrap(s.heading-startHeading_))>1) { cancel();message_="ROBOT_MOVED_RETRY";return true; }
  if(now-started_>4000) { cancel();message_="STALE_OR_MISSING_SENSOR_UPDATES";return true; }
  // Let existing ultrasonic median histories settle after the operator's rotation.
  if(now-started_<350) { for(uint8_t i=0;i<2;++i) lastRange_[i]=s.rangeSequence[i];lastLight_=s.lightSequence;return false; }
  if(s.lightSequence!=lastLight_&&lightSeen_<16) {
    lastLight_=s.lightSequence;
    if(s.lightValid) { for(uint8_t i=0;i<3;++i) lightSum_[i]+=s.light[i];++lightSeen_; }
  }
  for(uint8_t i=0;i<2;++i) if(s.rangeSequence[i]!=lastRange_[i]&&rangeSeen_[i]<8) {
    lastRange_[i]=s.rangeSequence[i];++rangeSeen_[i];
    if(s.range[i]>=20&&s.range[i]<=4000) {
      ++rangeGood_[i];rangeSum_[i]+=s.range[i];if(s.range[i]<low_[i]) low_[i]=s.range[i];if(s.range[i]>high_[i]) high_[i]=s.range[i];
    }
    headingSum_+=wrap(s.heading-startHeading_);++headings_;
  }
  bool done=kind_==CaptureKind::DARK?lightSeen_>=16:rangeSeen_[0]>=8&&rangeSeen_[1]>=8;
  if(kind_==CaptureKind::POINT) done=done&&lightSeen_>=16;
  if(!done) return false;
  CaptureKind completed=kind_;kind_=CaptureKind::NONE;
  if(completed==CaptureKind::DARK) {
    for(uint8_t i=0;i<3;++i) cfg_.ambient[i]=lightSum_[i]/float(lightSeen_);
    cfg_.ambientKnown=true;invalidate();message_="AMBIENT_CAPTURED_TURN_LIGHTS_ON";return true;
  }
  for(uint8_t i=0;i<2;++i) if(rangeGood_[i]>0&&(rangeGood_[i]<6||high_[i]-low_[i]>25)) { message_="UNSTABLE_RANGE_RETRY";return true; }
  SurveySample sample;sample.x=x_;sample.y=y_;sample.yaw=wrap(startHeading_+offset_+(headings_?headingSum_/headings_:0));sample.ms=now;
  for(uint8_t i=0;i<2;++i) sample.range[i]=rangeGood_[i]>=6?(rangeSum_[i]+rangeGood_[i]/2)/rangeGood_[i]:-1;
  sample.optical=completed==CaptureKind::POINT;
  for(uint8_t i=0;i<3;++i) sample.light[i]=lightSeen_?(lightSum_[i]+lightSeen_/2)/lightSeen_:0;
  samples_[count_++]=sample;invalidate();message_="SAMPLE_CAPTURED";return true;
}
bool Survey::endpoint(uint8_t frame,uint8_t sensor,float& x,float& y) const {
  const SurveySample& s=samples_[frame];if(s.range[sensor]<20) return false;
  const SensorMount& m=cfg_.mount[sensor];float a=s.yaw*0.01745329252f,beam=a+(sensor==0?0.7853981634f:-0.7853981634f);
  x=s.x+m.forward*cosf(a)-m.left*sinf(a)+s.range[sensor]*cosf(beam);
  y=s.y+m.forward*sinf(a)+m.left*cosf(a)+s.range[sensor]*sinf(beam);return true;
}
void Survey::solveWalls() {
  Endpoint points[SURVEY_CAPACITY*2];uint8_t total=0;
  for(uint8_t i=0;i<count_;++i) for(uint8_t j=0;j<2;++j) {
    Endpoint& p=points[total++];p.valid=endpoint(i,j,p.x,p.y);
  }
  for(uint8_t i=0;i<3;++i) fitWall(i,points,total);
}
void Survey::fitWall(uint8_t id,const Endpoint* points,uint8_t total) {
  SurveyWall result;float prior=id==0?0:id==1?FIELD_WIDTH_MM/2:-FIELD_WIDTH_MM/2;
  float best=prior;uint8_t bestCount=0;
  // Cache transformed endpoints once: fitting must not repeat trig inside consensus loops.
  for(uint8_t n=0;n<total;++n) {
    const Endpoint& p=points[n];if(!p.valid) continue;float candidate=id==0?p.x:p.y;
    if(fabsf(candidate-prior)>120) continue;
    uint8_t hits=0;
    for(uint8_t j=0;j<total;++j) if(points[j].valid&&fabsf((id==0?points[j].x:points[j].y)-candidate)<=25) ++hits;
    if(hits>bestCount) { bestCount=hits;best=candidate; }
  }
  float sumA=0,sumB=0,sumBB=0,sumAB=0,min=1e9f,max=-1e9f;uint8_t hits=0;
  for(uint8_t n=0;n<total;++n) {
    const Endpoint& p=points[n];if(!p.valid) continue;float a=id==0?p.x:p.y,b=id==0?p.y:p.x;
    if(fabsf(a-best)>25) continue;
    sumA+=a;sumB+=b;sumBB+=b*b;sumAB+=a*b;++hits;if(b<min) min=b;if(b>max) max=b;
  }
  result.points=hits;
  if(hits>=6&&max-min>=150) {
    float variance=sumBB-sumB*sumB/hits;
    if(variance>10000) {
      result.slope=(sumAB-sumA*sumB/hits)/variance;result.intercept=(sumA-result.slope*sumB)/hits;
      float ss=0;
      for(uint8_t n=0;n<total;++n) {
        const Endpoint& p=points[n];if(!p.valid) continue;float a=id==0?p.x:p.y,b=id==0?p.y:p.x;
        if(fabsf(a-best)<=25) ss+=powf(a-result.intercept-result.slope*b,2);
      }
      result.rms=sqrtf(ss/hits);result.alongMin=min;result.alongMax=max;
      result.valid=result.rms<=20&&fabsf(result.slope)<=0.14f&&fabsf(result.intercept-prior)<=100;
    }
  }
  cfg_.wall[id]=result;
}
static bool lightUsable(const SurveySample& s,uint8_t id,float ambient,float yaw) {
  return s.optical&&fabsf(wrap(s.yaw-yaw))<=3&&s.light[id]<1018&&s.light[id]-ambient>=5;
}
void Survey::fitBeacon(uint8_t id) {
  BeaconFit result;fit_[id]=result;
  if(!cfg_.ambientKnown||!cfg_.opticsKnown) return;
  float yaw=0;bool haveYaw=false;
  for(uint8_t i=0;i<count_;++i) if(samples_[i].optical) { yaw=samples_[i].yaw;haveYaw=true;break; }
  if(!haveYaw) return;
  uint8_t used[SURVEY_CAPACITY],n=0;
  float sumX=0,sumY=0,sumXX=0,sumYY=0,sumXY=0,maxX=0;
  // Repeating a pose cannot pretend to add independent position information.
  for(uint8_t i=0;i<count_;++i) {
    const SurveySample& s=samples_[i];if(!lightUsable(s,id,cfg_.ambient[id],yaw)) continue;
    bool duplicate=false;
    for(uint8_t j=0;j<n;++j) if(hypotf(s.x-samples_[used[j]].x,s.y-samples_[used[j]].y)<50) duplicate=true;
    if(duplicate) continue;
    used[n++]=i;float x=s.x*.001f,y=s.y*.001f;sumX+=x;sumY+=y;sumXX+=x*x;sumYY+=y*y;sumXY+=x*y;if(x>maxX) maxX=x;
  }
  result.points=n;fit_[id]=result;if(n<6) return;
  float xx=sumXX-sumX*sumX/n,yy=sumYY-sumY*sumY/n,xy=sumXY-sumX*sumY/n;
  float eigen=.5f*(xx+yy-sqrtf((xx-yy)*(xx-yy)+4*xy*xy));
  if(eigen<0.02f) return; // A two-dimensional footprint is required.
  float logSignal[SURVEY_CAPACITY];
  for(uint8_t j=0;j<n;++j) logSignal[j]=logf(samples_[used[j]].light[id]-cfg_.ambient[id]);
  float wallC=0,wallSlope=0;
  if(id==0) { wallC=cfg_.wall[1].intercept*.001f;wallSlope=cfg_.wall[1].slope; }
  else if(id==2) { wallC=cfg_.wall[2].intercept*.001f;wallSlope=cfg_.wall[2].slope; }
  const float height=cfg_.height[id]*.001f,exponent=cfg_.exponent;
  float best=1e9f,bx=0,blog=0,binfo=0,second=1e9f,secondX=0;
  for(uint8_t seed=0;seed<(id==1?1:5);++seed) {
    float x=id==1?0:seed==0?0:seed==1?maxX*.5f:seed==2?maxX:seed==3?maxX+.5f:maxX*2;
    for(uint8_t iteration=0;iteration<24;++iteration) {
      float mean=0,meanJ=0;
      for(uint8_t j=0;j<n;++j) {
        const auto& s=samples_[used[j]];float dx=x-s.x*.001f,dy=(id==1?0:wallC+wallSlope*x)-s.y*.001f;
        float d2=fmaxf(.0025f,dx*dx+dy*dy+height*height);
        mean+=logSignal[j]+.5f*exponent*logf(d2);meanJ+=exponent*(dx+wallSlope*dy)/d2;
      }
      mean/=n;meanJ/=n;float error=0,gradient=0,info=0;
      for(uint8_t j=0;j<n;++j) {
        const auto& s=samples_[used[j]];float dx=x-s.x*.001f,dy=(id==1?0:wallC+wallSlope*x)-s.y*.001f;
        float d2=fmaxf(.0025f,dx*dx+dy*dy+height*height),e=logSignal[j]+.5f*exponent*logf(d2)-mean;
        float derivative=exponent*(dx+wallSlope*dy)/d2-meanJ;error+=e*e;gradient+=derivative*e;info+=derivative*derivative;
      }
      if(id==1||iteration==23||info<1e-7f||fabsf(gradient)<1e-7f) {
        float score=sqrtf(error/n);
        if(score<best) { if(fabsf(x-bx)>.08f) { second=best;secondX=bx; }best=score;bx=x;blog=mean;binfo=info; }
        else if(fabsf(x-bx)>.08f&&score<second) { second=score;secondX=x; }
        break;
      }
      float step=clamp(gradient/info,.25f),next=fmaxf(0,fminf(10,x-step));
      if(fabsf(next-x)<.00001f) iteration=22;
      x=next;
    }
  }
  float signal=expf(blog);float sensitivity=id==1?0:2000*fmaxf(.02f,best)/sqrtf(fmaxf(1e-9f,binfo));
  float by=id==1?0:wallC+wallSlope*bx,sum=0,maxError=0;
  for(uint8_t j=0;j<n;++j) {
    const auto& s=samples_[used[j]];float dx=bx-s.x*.001f,dy=by-s.y*.001f;
    float geometric=sqrtf(dx*dx+dy*dy+height*height),inferred=powf(signal/(s.light[id]-cfg_.ambient[id]),1/exponent);
    float error=1000*fabsf(inferred-geometric);sum+=error*error;if(error>maxError) maxError=error;
  }
  result.model.x=bx*1000;result.model.y=by*1000;result.model.height=cfg_.height[id];result.model.ambient=cfg_.ambient[id];
  result.model.signal=signal;result.model.distance=1000;result.model.exponent=exponent;result.rms=sqrtf(sum/n);result.sensitivity=sensitivity;
  result.valid=signal>=5&&signal<=1023&&result.rms<=35&&maxError<=80&&sensitivity<=75&&!(second<best+.005f&&fabsf(secondX-bx)>.08f);
  fit_[id]=result;
}
bool Survey::solve() {
  if(busy()||!cfg_.mountsKnown||count_<6) { message_="NEED_SIX_CAPTURES";return false; }
  solving_=true;
  solveWalls();
  cfg_.measuredWidth=cfg_.wall[1].intercept-cfg_.wall[2].intercept;
  cfg_.wallsValid=cfg_.wall[0].valid&&cfg_.wall[1].valid&&cfg_.wall[2].valid&&fabsf(cfg_.measuredWidth-FIELD_WIDTH_MM)<=60;
  if(cfg_.wallsValid) for(uint8_t i=0;i<3;++i) fitBeacon(i);
  solving_=false;solved_=true;message_=cfg_.wallsValid?"WALLS_SOLVED_CHECK_LIGHT_FITS":"INSUFFICIENT_OR_INCONSISTENT_WALL_RETURNS";
  return true;
}
bool Survey::apply(Profile& p) {
  if(!solved_||busy()||!cfg_.wallsValid) { message_="SOLVE_VALID_WALLS_FIRST";return false; }
  bool mountChanged=!p.survey.mountsKnown;
  for(uint8_t i=0;i<2;++i) if(fabsf(p.survey.mount[i].forward-cfg_.mount[i].forward)>1||fabsf(p.survey.mount[i].left-cfg_.mount[i].left)>1) mountChanged=true;
  p.survey=cfg_;
  if(mountChanged) for(auto& point:p.point) point.samples=0;
  uint8_t opticalSamples=0;for(uint8_t i=0;i<count_;++i) if(samples_[i].optical) ++opticalSamples;
  if(cfg_.ambientKnown&&cfg_.opticsKnown&&opticalSamples>=6) {
    p.beaconPositionMask=0;
    for(uint8_t i=0;i<3;++i) if(fit_[i].valid) { p.beacon[i]=fit_[i].model;p.beaconPositionMask|=1<<i; }
  }
  session_=false;message_="APPLIED_IN_RAM_USE_SAVE_FOR_EEPROM";return true;
}
}
