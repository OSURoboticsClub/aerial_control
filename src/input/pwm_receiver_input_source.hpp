#ifndef PWM_RECEIVER_INPUT_SOURCE_HPP_
#define PWM_RECEIVER_INPUT_SOURCE_HPP_

#include "input/input_source.hpp"

class PWMReceiverInputSource : public InputSource {
public:
  ControllerInput read() override;
};

#endif
