#include "PID.hpp"

PID_Calibration::PID_Calibration(float kp, float ki, float kd)
    : kp(kp),
      ki(ki),
      kd(kd),
      target(.0f),
      actual(.0f),
      output(.0f),
      time_delta(1.0f),
      previous_error(.0f),
      integral(.0f),
      error(.0f),
      d(.0f) {
}

void PID_Calibration::setParam(float kp, float ki, float kd) {
    setKp(kp);
    setKi(ki);
    setKd(kd);
}

void PID_Calibration::setOutputLimits(float min, float max) {
    setOutputMin(min);
    setOutputMax(max);
}

void PID_Calibration::setReal(float real) {
    actual = real;
}

void PID_Calibration::setTarget(float _target) {
    target = _target;
}

float PID_Calibration::compute(void) {
    error = target - actual;                      // 误差计算
    integral += error;                            // 积分计算
    d = (error - previous_error) / time_delta;    // 微分计算
    output = kp * error + ki * integral + kd * d; // pid计算
    previous_error = error;                       // 误差记录
    if (output <= outputMin)
        output = outputMin;
    if (output >= outputMax)
        output = outputMax;

    return output;
}

float PID_Calibration::compute(float real) {
    setReal(real);
    return compute();
}

PID_Add::PID_Add(float kp, float ki, float kd)
    : kp_add(kp), ki_add(ki), kd_add(kd),
      set_target(0),
      actual(0),
      output(0),
      last_error(0),
      error(0),
      next_error(0) {
}

void PID_Add::setParam(float kp, float ki, float kd) {
    setKp(kp);
    setKi(ki);
    setKd(kd);
}

void PID_Add::setOutputLimits(float min, float max) {
    setOutputMin(min);
    setOutputMax(max);
}

float PID_Add::compute(void) {
    error = set_target - actual;
    output += kp_add * (error - next_error) + ki_add * error + kd_add * (error - 2 * next_error + last_error);
    if (output <= outputMin)
        output = outputMin;
    if (output >= outputMax)
        output = outputMax;
    last_error = next_error;
    next_error = error;
    return output;
}
