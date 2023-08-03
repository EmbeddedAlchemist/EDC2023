#include "Servo.hpp"

Servo::Servo(ExLib::PWM_Channel &pwmChannel, float minAngel, float maxAngel, ExLib::TimeInterval minPulse, ExLib::TimeInterval maxPulse, ExLib::TimeInterval cycle)
    : pwmChannel(pwmChannel),
      minAngel(minAngel), maxAngel(maxAngel),
      dutyMin((float)minPulse.us / cycle.us),
      dutyMax((float)maxPulse.us / cycle.us) {
    pwmChannel._setCycle(cycle);
}

void Servo::setAngel(float angel) {
    pwmChannel.setDuty((angel - minAngel) * (dutyMax - dutyMin) / (maxAngel - minAngel) + dutyMin);
}
