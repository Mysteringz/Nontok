#include "survey.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace core;
static uint32_t now=0;
static Sensors sensors;
static int16_t ray(float x,float y,float yaw,uint8_t i) {
  float a=yaw*.01745329252f,sx=x+90*cosf(a)-(i==0?70:-70)*sinf(a),sy=y+90*sinf(a)+(i==0?70:-70)*cosf(a);
  float angle=a+(i==0?.7853981634f:-.7853981634f),dx=cosf(angle),dy=sinf(angle),best=1e9f;
  if(dx<-.001f) { float r=-sx/dx,hit=sy+r*dy;if(r>0&&fabsf(hit)<=750) best=r; }
  for(int sign:{-1,1}) if(dy*sign>.001f) {
    float r=(750*sign-sy)/dy,hit=sx+r*dx;if(r>0&&hit>=0&&hit<=1200&&r<best) best=r;
  }
  return best>4000?-1:(int16_t)lroundf(best);
}
static void scene(float x,float y,float yaw,bool dark=false) {
  sensors.imuValid=true;sensors.gyroRate=0;sensors.heading=yaw;sensors.lightValid=true;
  for(uint8_t i=0;i<2;++i) sensors.range[i]=ray(x,y,yaw,i);
  float bx[]={180,0,250},by[]={750,0,-750},height[]={100,120,80},gain[]={180,40,150},ambient[]={7,9,12};
  for(uint8_t i=0;i<3;++i) {
    float d2=(x-bx[i])*(x-bx[i])+(y-by[i])*(y-by[i])+height[i]*height[i];
    sensors.light[i]=lroundf(ambient[i]+(dark?0:gain[i]*1000000/d2));
  }
}
static void take(Survey& survey,CaptureKind kind) {
  assert(survey.capture(kind,now,sensors));
  for(uint32_t t=20;t<=2000&&survey.busy();t+=20) {
    now+=20;++sensors.lightSequence;
    if(t%80==0) { ++sensors.rangeSequence[0];++sensors.rangeSequence[1]; }
    survey.update(now,sensors);
  }
  assert(!survey.busy());
}
static void walls(Survey& s) {
  for(int yaw=-180;yaw<180;yaw+=15) { scene(600,0,yaw);take(s,CaptureKind::WALL); }
}
int main() {
  Survey survey;scene(600,0,180);
  assert(!survey.begin(600,0,180,sensors));
  assert(!survey.mounts(std::numeric_limits<float>::quiet_NaN(),0,0,0));
  assert(survey.mounts(90,70,90,-70));assert(survey.begin(600,0,180,sensors));
  assert(!survey.solve());assert(!survey.setPose(-1,0));assert(!survey.setPose(600,800));
  walls(survey);assert(survey.count()==24);assert(survey.solve());
  assert(survey.calibration().wallsValid);assert(fabsf(survey.calibration().measuredWidth-1500)<3);
  for(const auto& wall:survey.calibration().wall) { assert(wall.valid&&wall.points>=6&&wall.rms<2); }
  scene(600,0,180,true);take(survey,CaptureKind::DARK);assert(survey.calibration().ambientKnown);
  assert(survey.optics(100,120,80,2));
  for(int x:{350,650,950}) for(int y:{-300,0,300}) {
    assert(survey.setPose(x,y));scene(x,y,180);take(survey,CaptureKind::POINT);
  }
  assert(survey.count()==33);assert(survey.solve());assert(survey.calibration().wallsValid);
  const float expectedX[]={180,0,250};
  for(uint8_t i=0;i<3;++i) {
    const auto& b=survey.beacon(i);
    std::cout<<"Beacon "<<int(i)<<" x="<<b.model.x<<" rms="<<b.rms<<" sensitivity="<<b.sensitivity<<" valid="<<b.valid<<'\n';
    assert(b.valid&&fabsf(b.model.x-expectedX[i])<15&&b.rms<10);
  }
  Profile profile;defaultProfile(profile);assert(survey.apply(profile));
  assert(!survey.session()&&profile.survey.wallsValid&&profile.beaconPositionMask==7);
  assert(profile.point[CENTER].samples==0); // Moving/changing mounts requires new local references.
  assert(!profileValid(profile)); // Survey does not fabricate paper and box captures.
  assert(survey.begin(600,0,180,sensors));scene(600,0,180);
  assert(survey.capture(CaptureKind::WALL,now,sensors));sensors.gyroRate=2;
  assert(survey.update(now+20,sensors));assert(!survey.busy()&&survey.count()==0);
  sensors.gyroRate=0;assert(survey.capture(CaptureKind::WALL,now,sensors));
  assert(survey.update(now+4001,sensors));assert(!survey.busy()&&survey.count()==0);
  // A circle scan and repeated optical pose do not constrain a light's position.
  scene(600,0,180);walls(survey);
  for(int i=0;i<7;++i) { assert(survey.setPose(600,0));scene(600,0,180);take(survey,CaptureKind::POINT); }
  assert(survey.solve()&&survey.calibration().wallsValid);
  for(uint8_t i=0;i<3;++i) assert(!survey.beacon(i).valid);
  assert(survey.apply(profile)&&profile.beaconPositionMask==0);
  survey.clear();assert(survey.count()==0&&!survey.session());assert(!survey.apply(profile));
  // A room-like width is never synthesized from no-echo returns.
  scene(600,0,180);assert(survey.begin(600,0,180,sensors));
  for(int i=0;i<6;++i) { sensors.range[0]=sensors.range[1]=-1;take(survey,CaptureKind::WALL); }
  assert(survey.solve()&&!survey.calibration().wallsValid&&!survey.apply(profile));
  std::cout<<"Guided survey: geometry, optical fitting, stale/moving/duplicate/insufficient data and apply guards passed\n";
}
