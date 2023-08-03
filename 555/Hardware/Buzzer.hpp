#pragma once

#include "ExLib_FreeRTOS.hpp"
#include "ExLib_PWM.hpp"

class Buzzer {
  private:
    struct BuzzerTaskData {
        ExLib::PWM_Channel &channel;
        ExLib::Frequency freq;
        ExLib::TimeInterval interval;
    };

    static ExLib::Task *buzzerTask;
    static ExLib::Queue<BuzzerTaskData> *buzzerQueue;
    static void buzzerTaskFunction(void *taskQueue);

    ExLib::PWM_Channel &channe;

  public:
    Buzzer(ExLib::PWM_Channel &channel);
    void beep(ExLib::Frequency freq, ExLib::TimeInterval time);
};