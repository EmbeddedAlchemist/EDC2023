#pragma once

#include <stdint.h>

class PID {
  private:
    std::uint8_t buffer[72];
    void *handler;

  public:
    PID(void);
    PID(float kp, float ki, float kd);

    void setKp(float kp);
    void setKi(float ki);
    void setKd(float kd);
    void setParams(float kp, float ki, float kd);

    void setOutputMin(float min);
    void setOutputMax(float max);
    void setOutputLimits(float min, float max);

    void setReal(float real);
    void setTarget(float target);
    float getOutput(void);

    float compute(void);
};