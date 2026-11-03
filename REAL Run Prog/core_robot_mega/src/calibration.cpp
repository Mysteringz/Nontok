#include "calibration.h"
#include <math.h>
namespace core {
static bool ranges(const Sensors& s) { return s.range[0]>=80&&s.range[1]>=80&&s.imuValid&&fabsf(s.gyroRate)<1.5f; }
bool captureReference(Profile& p,uint8_t id,const Sensors& s,float x,float y,float yaw) {
  if(id>=POINTS||!ranges(s)||!isfinite(x)||!isfinite(y)||!isfinite(yaw)||x<=0||x>10000||fabsf(y)>10000||fabsf(yaw)>360) return false;
  Reference r;r.x=x;r.y=y;r.yaw=wrap(yaw);r.range[0]=s.range[0];r.range[1]=s.range[1];r.samples=1;r.capturedGyro=s.heading;
  for(uint8_t i=0;i<3;++i) r.light[i]=s.light[i];
  p.point[id]=r;return true;
}
bool captureDerivative(Profile& p,uint8_t id,uint8_t axis,const Sensors& s,float offset) {
  if(id>=POINTS||axis>2||!ranges(s)||!(p.point[id].samples&1)||!isfinite(offset)) return false;
  if(axis==2?(fabsf(offset)<2||fabsf(offset)>10):(fabsf(offset)<20||fabsf(offset)>80)) return false;
  Reference& r=p.point[id];
  for(uint8_t i=0;i<2;++i) {
    float derivative=(s.range[i]-r.range[i])/offset;
    if(!isfinite(derivative)||fabsf(derivative)>100) return false;
  }
  for(uint8_t i=0;i<2;++i) r.jacobian[i][axis]=(s.range[i]-r.range[i])/offset;
  r.samples|=1<<(axis+1);return true;
}
bool captureColor(Profile& p,Color c,const Sensors& s) {
  if(c==Color::UNKNOWN||!s.colorValid||s.rgba[3]<20||s.rgba[3]>60000) return false;
  float sum=float(s.rgba[0])+s.rgba[1]+s.rgba[2];if(sum<20) return false;
  uint8_t i=c==Color::RED?0:1;
  for(uint8_t j=0;j<3;++j) p.color[i][j]=s.rgba[j]/sum;
  p.colorMask|=1<<i;return true;
}
}
