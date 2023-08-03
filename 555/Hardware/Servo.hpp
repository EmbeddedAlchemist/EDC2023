#pragma once

#include "ExLib_PWM.hpp"
#include "ExLib_Units.hpp"

class Servo {
  private:
    ExLib::PWM_Channel &pwmChannel;
    float maxAngel;
    float minAngel;
    float dutyMin, dutyMax;

  public:
    Servo(void) = delete;
    Servo(ExLib::PWM_Channel &pwmChannel,
          float minAngel = 0,
          float maxAngel = 180,
          ExLib::TimeInterval minPulse = ExLib::TimeInterval(500),
          ExLib::TimeInterval maxPulse = ExLib::TimeInterval(2500),
          ExLib::TimeInterval cycle = ExLib::TimeInterval(20000));
    void setAngel(float angel);
};