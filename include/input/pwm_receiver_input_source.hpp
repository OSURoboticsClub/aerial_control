#ifndef PWM_RECEIVER_INPUT_SOURCE_H_
#define PWM_RECEIVER_INPUT_SOURCE_H_

#include <input/input_source.hpp>

class PWMReceiverInputSource : public InputSource {
public:
  struct controller_input_t read();
};

#endif
