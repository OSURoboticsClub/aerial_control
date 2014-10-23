#ifndef MOTOR_MAPPER_HPP_
#define MOTOR_MAPPER_HPP_

#include <config.hpp>

struct motor_output_t {
  float outputs[NUM_ROTORS];
};

class MotorMapper {
public:
  MotorMapper();

  struct motor_output_t run(struct controller_output_t& input);
};

#endif
