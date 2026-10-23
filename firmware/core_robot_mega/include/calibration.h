#pragma once
#include "control.h"
namespace core {
bool captureReference(Profile&,uint8_t,const Sensors&,float x,float y,float yaw);
bool captureDerivative(Profile&,uint8_t,uint8_t,const Sensors&,float offset);
bool captureColor(Profile&,Color,const Sensors&);
}
bool profileLoad(core::Profile&);
bool profileSave(const core::Profile&);
