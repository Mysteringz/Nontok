#include "control.h"
#include <math.h>
namespace core {
static float limit(float x,float n) { return fmaxf(-n,fminf(n,x)); }
float wrap(float d) { while(d>180) d-=360; while(d<=-180) d+=360; return d; }
void defaultProfile(Profile& p) {
  p=Profile();
  p.beacon[0].y=FIELD_WIDTH_MM/2; p.beacon[2].y=-FIELD_WIDTH_MM/2;
  // Side beacon X is unknown: zero is a placeholder, never a surveyed location.
  p.beaconPositionMask=2; // Only the middle beacon is defined as the origin.
  p.point[CENTER].x=300;
}
bool beaconGeometryValid(const Profile& p) {
  if(p.beaconPositionMask!=7) return false;
  for(uint8_t i=0;i<3;++i) {
    const Beacon& b=p.beacon[i];
    if(!isfinite(b.x)||!isfinite(b.y)||fabsf(b.x)>10000||fabsf(b.y)>10000) return false;
    for(uint8_t j=0;j<i;++j) if(hypotf(b.x-p.beacon[j].x,b.y-p.beacon[j].y)<50) return false;
  }
  return true;
}
bool profileValid(const Profile& p) {
  if(!beaconGeometryValid(p)) return false;
  if (!(p.positionTolerance>=5 && p.positionTolerance<=100 && p.yawTolerance>=0.5f && p.yawTolerance<=5 &&
        p.rangeTolerance>=3 && p.rangeTolerance<=50 && p.beaconResidualLimit>=5 && p.beaconResidualLimit<=300 &&
        p.localRadius>=30 && p.localRadius<=200 && p.guard>=30 && p.guard<=300)) return false;
  for (const Beacon& b:p.beacon) {
    if (!isfinite(b.x)||!isfinite(b.y)||!isfinite(b.height)||!isfinite(b.ambient)||!isfinite(b.signal)||
        !isfinite(b.distance)||!isfinite(b.exponent)||b.height<0||b.height>3000||b.ambient<0||b.ambient>1000||
        b.signal<5||b.signal>1023||b.distance<50||b.distance>5000||b.exponent<0.5f||b.exponent>5) return false;
  }
  if(p.colorMask!=3) return false;
  float separation=0;
  for(uint8_t i=0;i<2;++i) for(float c:p.color[i]) if(!isfinite(c)||c<0||c>1) return false;
  for(uint8_t j=0;j<3;++j) separation+=powf(p.color[0][j]-p.color[1][j],2);
  if(separation<0.04f) return false;
  for(const Reference& r:p.point) {
    if(r.samples!=15||!isfinite(r.x)||!isfinite(r.y)||!isfinite(r.yaw)||r.x<=0||fabsf(r.y)>10000||r.x>10000) return false;
    float yawSensitivity=0;
    for(uint8_t i=0;i<2;++i) {
      if(!isfinite(r.range[i])||r.range[i]<p.guard||r.range[i]>4000) return false;
      for(uint8_t j=0;j<3;++j) if(!isfinite(r.jacobian[i][j])||fabsf(r.jacobian[i][j])>100) return false;
      yawSensitivity+=r.jacobian[i][2]*r.jacobian[i][2];
    }
    if(yawSensitivity<0.25f) return false; // Unobservable ultrasonic yaw cannot count as alignment.
  }
  for(uint8_t i=TURN_LEFT;i<=TURN_RIGHT;++i) {
    if(hypotf(p.point[i].x-p.point[CENTER].x,p.point[i].y-p.point[CENTER].y)>5 ||
       fabsf(wrap(p.point[i].yaw-p.point[CENTER].yaw-(i==TURN_LEFT?90:-90)))>1) return false;
  }
  return true;
}
Color classify(const Sensors& s,const Profile& p) {
  if(!s.colorValid||p.colorMask!=3||s.rgba[3]<20||s.rgba[3]>60000) return Color::UNKNOWN;
  float total=float(s.rgba[0])+s.rgba[1]+s.rgba[2]; if(total<20) return Color::UNKNOWN;
  float d[2]={0,0};
  for(uint8_t i=0;i<2;++i) for(uint8_t j=0;j<3;++j) d[i]+=powf(s.rgba[j]/total-p.color[i][j],2);
  uint8_t best=d[0]<d[1]?0:1;
  if(d[best]>0.035f || d[1-best]-d[best]<0.025f) return Color::UNKNOWN;
  return best==0?Color::RED:Color::GREEN;
}
const char* colorName(Color c) { return c==Color::RED?"RED":c==Color::GREEN?"GREEN":"UNKNOWN"; }
const char* pointName(uint8_t p) { static const char* names[]={"CENTER","TURN_LEFT","TURN_RIGHT","BOX_LEFT","BOX_RIGHT"}; return p<POINTS?names[p]:"NONE"; }
void Localizer::resetHeading(float world,float gyro) { offset_=wrap(world-gyro); headingReady_=true; }
// General measured beacon coordinates, including side lights forward of the back wall.
// +X is the arena side of the back-wall origin, not an assumed common beacon line.
static bool opticalPosition(const Sensors& s,const Profile& p,float& x,float& y,float& residual) {
  if(!s.lightValid||!beaconGeometryValid(p)) return false;
  float distance[3];
  for(uint8_t i=0;i<3;++i) {
    const Beacon& b=p.beacon[i]; float signal=s.light[i]-b.ambient;
    if(signal<5||s.light[i]>=1018||b.signal<5||b.distance<50||b.exponent<0.5f) return false;
    float slant=b.distance*powf(b.signal/signal,1/b.exponent);
    if(!isfinite(slant)||slant<=b.height||slant>12000) return false;
    distance[i]=sqrtf(slant*slant-b.height*b.height);
  }
  // Multi-start Gauss Newton; reject equally good, spatially distinct solutions.
  float best=1e9f,bx=0,by=0,second=1e9f,sx=0,sy=0;
  for(uint8_t seed=0;seed<7;++seed) {
    float px=seed==0?fmaxf(30,x):((seed%3)+1)*500.0f;
    float py=seed==0?y:((int)(seed/3)-1)*600.0f;
    bool solvable=true;
    for(uint8_t k=0;k<16;++k) {
      float aa=0,ab=0,bb=0,ax=0,ay=0;
      for(uint8_t i=0;i<3;++i) {
        float dx=px-p.beacon[i].x,dy=py-p.beacon[i].y,dd=fmaxf(1,hypotf(dx,dy));
        float jx=dx/dd,jy=dy/dd,e=distance[i]-dd;
        aa+=jx*jx; ab+=jx*jy; bb+=jy*jy; ax+=jx*e; ay+=jy*e;
      }
      float det=aa*bb-ab*ab;
      if(det<0.0001f) { solvable=false; break; }
      float dx=limit((bb*ax-ab*ay)/det,500),dy=limit((aa*ay-ab*ax)/det,500);
      px+=dx; py+=dy;
      if(px<1) px=1;
      if(hypotf(dx,dy)<0.1f) break;
    }
    if(!solvable||px<10||px>10000||fabsf(py)>10000) continue;
    float sum=0;
    for(uint8_t i=0;i<3;++i) sum+=powf(hypotf(px-p.beacon[i].x,py-p.beacon[i].y)-distance[i],2);
    float score=sqrtf(sum/3);
    if(score<best) {
      if(hypotf(px-bx,py-by)>150) { second=best; sx=bx; sy=by; }
      best=score; bx=px; by=py;
    } else if(hypotf(px-bx,py-by)>150 && score<second) { second=score; sx=px; sy=py; }
  }
  if(best>p.beaconResidualLimit || (second<best+5 && hypotf(sx-bx,sy-by)>150)) return false;
  x=bx; y=by; residual=best; return true;
}
static bool solve3(float a[3][4],float result[3]) {
  for(uint8_t i=0;i<3;++i) {
    uint8_t pivot=i;
    for(uint8_t j=i+1;j<3;++j) if(fabsf(a[j][i])>fabsf(a[pivot][i])) pivot=j;
    if(fabsf(a[pivot][i])<1e-8f) return false;
    for(uint8_t k=i;k<4;++k) { float v=a[i][k]; a[i][k]=a[pivot][k]; a[pivot][k]=v; }
    float d=a[i][i]; for(uint8_t k=i;k<4;++k) a[i][k]/=d;
    for(uint8_t j=0;j<3;++j) if(j!=i) { float f=a[j][i]; for(uint8_t k=i;k<4;++k) a[j][k]-=f*a[i][k]; }
  }
  for(uint8_t i=0;i<3;++i) result[i]=a[i][3];
  return true;
}
Pose Localizer::update(const Sensors& s,const Profile& p) {
  pose_.valid=false; pose_.ultrasonic=false; pose_.reference=-1; pose_.rangeError=-1;
  pose_.headingValid=headingReady_&&s.imuValid&&isfinite(s.heading);
  pose_.yaw=pose_.headingValid?wrap(s.heading+offset_):0;
  if(!opticalPosition(s,p,pose_.x,pose_.y,pose_.residual)) return pose_;
  pose_.valid=true;
  if(!pose_.headingValid||s.range[0]<0||s.range[1]<0) return pose_;
  float nearest=p.localRadius;
  for(uint8_t n=0;n<POINTS;++n) {
    const Reference& r=p.point[n]; float d=hypotf(pose_.x-r.x,pose_.y-r.y);
    if(r.samples==15&&d<nearest&&fabsf(wrap(pose_.yaw-r.yaw))<15) { pose_.reference=n; nearest=d; }
  }
  if(pose_.reference<0) return pose_;
  const Reference& r=p.point[pose_.reference];
  float delta[3]={pose_.x-r.x,pose_.y-r.y,wrap(pose_.yaw-r.yaw)};
  // Weighted local fusion: optical XY prior (40 mm), gyro yaw prior (2 degrees), ultrasound (8 mm).
  float normal[3][4]={{1/1600.0f,0,0,0},{0,1/1600.0f,0,0},{0,0,0.25f,0}};
  float sum=0;
  for(uint8_t i=0;i<2;++i) {
    float expected=r.range[i]; for(uint8_t j=0;j<3;++j) expected+=r.jacobian[i][j]*delta[j];
    float error=s.range[i]-expected; sum+=error*error;
    if(fabsf(error)>180) return pose_; // Wrong reflector/outside the local model: do not fuse.
    for(uint8_t j=0;j<3;++j) {
      normal[j][3]+=r.jacobian[i][j]*error/64;
      for(uint8_t k=0;k<3;++k) normal[j][k]+=r.jacobian[i][j]*r.jacobian[i][k]/64;
    }
  }
  float correction[3]; if(!solve3(normal,correction)||hypotf(correction[0],correction[1])>80||fabsf(correction[2])>8) return pose_;
  pose_.x+=correction[0]; pose_.y+=correction[1]; pose_.yaw=wrap(pose_.yaw+correction[2]);
  pose_.rangeError=sqrtf(sum/2); pose_.ultrasonic=true;
  return pose_;
}
}
