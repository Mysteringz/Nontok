#pragma once
#include "control.h"
namespace core {
constexpr uint8_t SURVEY_CAPACITY=40;
enum class CaptureKind:uint8_t { NONE,WALL,POINT,DARK };
struct SurveySample {
  float x=0,y=0,yaw=0;
  int16_t range[2]={-1,-1};
  uint16_t light[3]={0,0,0};
  uint32_t ms=0;
  bool optical=false;
};
struct BeaconFit { Beacon model;float rms=0,sensitivity=0;uint8_t points=0;bool valid=false; };
class Survey {
 public:
  void configure(const SurveyCalibration& cfg) { cfg_=cfg; }
  bool mounts(float lx,float ly,float rx,float ry);
  bool optics(float hl,float hm,float hr,float exponent);
  bool begin(float x,float y,float yaw,const Sensors& s);
  bool setPose(float x,float y);
  bool capture(CaptureKind kind,uint32_t now,const Sensors& s);
  bool update(uint32_t now,const Sensors& s); // True when a capture finishes or fails.
  void cancel();void end();void clear();
  bool solve();bool apply(Profile& profile);
  bool session() const { return session_; }
  bool busy() const { return kind_!=CaptureKind::NONE||solving_; }
  const char* message() const { return message_; }
  const char* phase() const;
  uint8_t count() const { return count_; }
  const SurveySample& sample(uint8_t i) const { return samples_[i]; }
  const SurveyCalibration& calibration() const { return cfg_; }
  const BeaconFit& beacon(uint8_t i) const { return fit_[i]; }
  bool solved() const { return solved_; }
  float poseX() const { return x_; } float poseY() const { return y_; }
  float worldYaw(float gyro) const { return wrap(gyro+offset_); }
 private:
  SurveyCalibration cfg_;
  SurveySample samples_[SURVEY_CAPACITY];
  BeaconFit fit_[3];
  uint8_t count_=0;
  float x_=0,y_=0,offset_=0,startHeading_=0;
  bool session_=false,solving_=false,solved_=false;
  CaptureKind kind_=CaptureKind::NONE;
  const char* message_="IDLE";
  uint32_t started_=0,lastRange_[2]={},lastLight_=0;
  int32_t rangeSum_[2]={};uint32_t lightSum_[3]={};
  uint8_t rangeSeen_[2]={},rangeGood_[2]={},lightSeen_=0;
  int16_t low_[2]={},high_[2]={};
  float headingSum_=0;uint16_t headings_=0;
  bool endpoint(uint8_t frame,uint8_t sensor,float& x,float& y) const;
  struct Endpoint { float x,y;bool valid; };
  void solveWalls();
  void fitWall(uint8_t id,const Endpoint* points,uint8_t total);
  void fitBeacon(uint8_t id);
  void invalidate();
};
}
