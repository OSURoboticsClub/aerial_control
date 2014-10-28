#include <motor/multirotor_quad_x_motor_mapper.hpp>

#include <controller/controller.hpp>

MultirotorQuadXMotorMapper::MultirotorQuadXMotorMapper() {
}

void MultirotorQuadXMotorMapper::init() {
  PWMMotorMapper::init();
}

void MultirotorQuadXMotorMapper::run(controller_output_t& input) {
  float outputs[4] = {
     1.0f * input.setpoints[0] + 1.0f * input.setpoints[1] + 1.0f * input.setpoints[2], // front left
    -1.0f * input.setpoints[0] + 1.0f * input.setpoints[1] - 1.0f * input.setpoints[2], // front right
    -1.0f * input.setpoints[0] - 1.0f * input.setpoints[1] + 1.0f * input.setpoints[2], // back right
     1.0f * input.setpoints[0] - 1.0f * input.setpoints[1] - 1.0f * input.setpoints[2]  // back left
  };

  for(int i = 0; i < 4; i++) {
    setMotorSpeed(i, 0.50f); // TODO: Fix this
  }
}
