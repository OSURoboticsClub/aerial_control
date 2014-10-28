#ifndef MOTOR_MAPPER_HPP_
#define MOTOR_MAPPER_HPP_

#include <controller/controller.hpp>

class MotorMapper {
public:
  virtual void init() =0;
  virtual void run(controller_output_t& input) =0;
};

#endif
