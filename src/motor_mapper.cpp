#include <motor_mapper.hpp>

#include <config.hpp>
#include <controller/controller.hpp>

MotorMapper::MotorMapper() {
}

struct motor_output_t MotorMapper::run(struct controller_output_t& input) {
  struct motor_output_t output = {
    .outputs = { 0 }
  };

  for(int i = 0; i < NUM_ROTORS; i++) {
    for(int axis = 0; axis < 3; axis++) {
      output.outputs[i] += input.setpoints[axis] * MIXER[i][axis];
    }
  }

  return output;
}
