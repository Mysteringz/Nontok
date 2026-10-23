#pragma once
#include <stdint.h>

namespace core {
constexpr uint8_t POINTS = 5;
constexpr float FIELD_WIDTH_MM = 1500; // User measured side-wall spacing.
enum PointId : uint8_t { CENTER, TURN_LEFT, TURN_RIGHT, BOX_LEFT, BOX_RIGHT };
enum class Color : uint8_t { UNKNOWN, RED, GREEN };
enum class Mode : uint8_t { DISARMED, IDLE, MANUAL, DEMO, TO_CENTER, READ_COLOR, TURN, TO_BOX, DOCK, FINISHED, FAULT };
struct Reference {
  float x=0, y=0, yaw=0;
  float range[2]={0,0};
  uint16_t light[3]={0,0,0};
  float capturedGyro=0;
  // Local range derivative per world-X mm, world-Y mm and CCW degree.
  float jacobian[2][3]={{0,0,0},{0,0,0}};
  uint8_t samples=0; // bit 0 base, bits 1..3 X/Y/YAW samples
};
struct Beacon {
  float x=0,y=0,height=0,ambient=0,signal=0,distance=1000,exponent=2;
};
struct SensorMount { float forward=0,left=0; };
// Wall equation: back x=intercept+slope*y; sides y=intercept+slope*x.
struct SurveyWall {
  float intercept=0,slope=0,rms=0,alongMin=0,alongMax=0;
  uint8_t points=0;bool valid=false;
};
struct SurveyCalibration {
  SensorMount mount[2];
  SurveyWall wall[3]; // back, +Y side, -Y side
  float ambient[3]={0,0,0},height[3]={0,0,0},exponent=2,measuredWidth=0;
  bool mountsKnown=false,ambientKnown=false,opticsKnown=false,wallsValid=false;
};
struct Profile {
  Beacon beacon[3]; // Independently measured left, middle, right beacon strengths.
  Reference point[POINTS];
  float color[2][3]={{0,0,0},{0,0,0}}; // Normalized red, green exemplars.
  float positionTolerance=20, yawTolerance=2, rangeTolerance=12;
  float beaconResidualLimit=100, localRadius=120, guard=80;
  uint8_t colorMask=0;
  SurveyCalibration survey;
  uint8_t beaconPositionMask=0; // Each bit requires explicit measured XY for that beacon.
};
struct Sensors {
  int16_t range[2]={-1,-1}; // FRONT LEFT +45 degrees, FRONT RIGHT -45 degrees.
  uint16_t light[3]={0,0,0};
  uint16_t rgba[4]={0,0,0,0};
  uint32_t colorSequence=0,rangeSequence[2]={0,0},lightSequence=0;
  float heading=0, gyroRate=0;
  bool imuValid=false,lightValid=false,colorValid=false;
};
struct Pose {
  float x=0,y=0,yaw=0,residual=0,rangeError=0;
  bool valid=false,headingValid=false,ultrasonic=false;
  int8_t reference=-1;
};
struct Settings { float manualGuard=80; };
typedef void (*Output)(float,float,float);
float wrap(float degrees);
void defaultProfile(Profile& profile);
bool beaconGeometryValid(const Profile& profile);
bool profileValid(const Profile& profile);
Color classify(const Sensors& sensors,const Profile& profile);
const char* colorName(Color color);
const char* pointName(uint8_t point);
const char* modeName(Mode mode);
class Localizer {
 public:
  void resetHeading(float worldHeading,float gyroHeading);
  void invalidate() { headingReady_=false; pose_=Pose(); }
  Pose update(const Sensors& sensors,const Profile& profile);
  Pose pose() const { return pose_; }
 private:
  Pose pose_;
  bool headingReady_=false;
  float offset_=0;
};
class Controller {
 public:
  Controller(Output output,bool motorsReady,bool sensorsReady,Profile& profile,const Settings& settings=Settings());
  bool arm(); void stop(); void fail(const char* reason);
  bool move(float vx,float vy,float wz,uint32_t durationMs,uint32_t now);
  bool demo(uint32_t now);
  bool start(uint32_t now,const Sensors& sensors,const Pose& pose);
  void tick(uint32_t now,const Sensors& sensors,const Pose& pose);
  Mode mode() const { return mode_; }
  const char* fault() const { return fault_; }
  float vx() const { return vx_; } float vy() const { return vy_; } float wz() const { return wz_; }
  bool armed() const;
  Color decision() const { return decision_; }
  int8_t target() const { return target_; }
 private:
  Output output_; bool motorsReady_,sensorsReady_; Profile& profile_; Settings cfg_;
  Mode mode_=Mode::DISARMED; const char* fault_="NONE";
  float vx_=0,vy_=0,wz_=0,turnGyro_=0;
  uint32_t entered_=0,started_=0,duration_=0,lastTick_=0,stableSince_=0,lastColorSequence_=0;
  bool stable_=false; uint8_t demoStep_=0,votes_=0; Color vote_=Color::UNKNOWN,decision_=Color::UNKNOWN;
  int8_t target_=-1;
  void drive(float,float,float); void enter(Mode,uint32_t);
  bool stableFor(bool,uint32_t,uint32_t);
  void navigate(const Pose&,const Reference&,bool fine);
  bool atReference(const Sensors&,const Pose&,uint8_t id) const;
};
}
