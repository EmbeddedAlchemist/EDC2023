#include "PID.hpp"
#include "Libraries/pid.h"

#include "ExLib_Exception.hpp"

PID::PID(void) {
    handler = PID_init(buffer, sizeof(buffer));
    if (handler == nullptr)
        ExLib::Exception::raiseException("Fail to create PID object");
}

PID::PID(float kp, float ki, float kd)
    : PID() {
    setParams(kp, ki, kd);
}

void PID::setKp(float kp) {
    PID_setKp((PID_Handle)handler, kp);
}

void PID::setKi(float ki) {
    PID_setKi((PID_Handle)handler, ki);
}

void PID::setKd(float kd) {
    PID_setKd((PID_Handle)handler, kd);
}

void PID::setParams(float kp, float ki, float kd) {
    setKp(kp);
    setKi(ki);
    setKd(kd);
}

void PID::setOutputMin(float min) {
    PID_setOutMin((PID_Handle)handler, min);
}

void PID::setOutputMax(float max) {
    PID_setOutMax((PID_Handle)handler, max);
}

void PID::setOutputLimits(float min, float max) {
    setOutputMin(min);
    setOutputMax(max);
}

void PID::setReal(float real) {
    PID_setFbackValue((PID_Handle)handler, real);
}

void PID::setTarget(float target) {
    PID_setRefValue((PID_Handle)handler, target);
}

float PID::getOutput(void) {
   // return PID_getFfwdValue()
}

float PID::compute(void) {
    
}
