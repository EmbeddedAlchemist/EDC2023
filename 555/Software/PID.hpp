#pragma once

#include "ExLib_Units.hpp"
#include <stdint.h>

class PID_Calibration {
  private:
    float kp;             // 比例
    float ki;             // 积分
    float kd;             // 微分
    float actual;         // 实际值
    float target;         // 目标值
    float time_delta;     // 时间
    float previous_error; // 过去的误差
    float integral;       // 累计积分
    float output;         // 输出值
    float error;          // 现在的误差
    float d;              // 微分的值

    float outputMax;
    float outputMin;

  public:
    PID_Calibration(float kp = .0f, float ki = .0f, float kd = .0f);

    inline void setKp(float kp) { this->kp = kp; }
    inline void setKi(float ki) { this->ki = ki; }
    inline void setKd(float kd) { this->kd = kd; }
    void setParam(float kp, float ki, float kd);

    inline void setOutputMax(float max) { outputMax = max; }
    inline void setOutputMin(float min) { outputMin = min; }
    void setOutputLimits(float min, float max);

    inline void setTimeInterval(ExLib::TimeInterval inteval) { time_delta = inteval.us; }

    void setReal(float real);
    void setTarget(float target);
    float compute(void);
    float compute(float Real);

    inline float getOutput(void) { return output; }
    inline float getReal(void) { return actual; }
    inline float getTarget(void) { return target; }
};

class PID_Add {
  private:
    float kp_add;
    float ki_add;
    float kd_add;

    float set_target;
    float actual;
    float output;

    float last_error;
    float error;
    float next_error;

    float outputMax;
    float outputMin;

  public:
    PID_Add(float kp = .0f, float ki = .0f, float kd = .0f);

    inline void setKp(float kp) { this->kp_add = kp; }
    inline void setKi(float ki) { this->ki_add = ki; }
    inline void setKd(float kd) { this->kd_add = kd; }
    void setParam(float kp, float ki, float kd);

    inline void setOutputMax(float max) { outputMax = max; }
    inline void setOutputMin(float min) { outputMin = min; }
    void setOutputLimits(float min, float max);

    // inline void setTimeInterval(ExLib::TimeInterval inteval) { time_delta = inteval.us; }

    inline void setReal(float real) { actual = real; }
    inline void setTarget(float target) { set_target = target; }
    float compute(void);
    float compute(float Real);

    inline float getOutput(void) { return output; }
    inline float getReal(void) { return actual; }
    inline float getTarget(void) { return set_target; }
};