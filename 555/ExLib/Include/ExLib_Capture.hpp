#pragma once

#include "ExLib_CallbackFunction.hpp"
#include "ExLib_GPIO.hpp"


namespace ExLib {


class Capturer {
  public:
    virtual void registerCaptureChannel(std::uint32_t channel, GPIO_Pin pinName, GPIO_State edge);
};

class Capture_Channel {
  private:
    CallbackFunction *callback;

  public:
    
};
} // namespace ExLib