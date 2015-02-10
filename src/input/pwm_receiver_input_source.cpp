#include "input/pwm_receiver_input_source.hpp"

ControllerInput PWMReceiverInputSource::read() {
  ControllerInput input {
    .valid = true,
    .roll = 0.0f,
    .pitch = 0.0f,
    .yaw = 0.0f,
    .throttle = 0.0f
  };

  return input;
}
