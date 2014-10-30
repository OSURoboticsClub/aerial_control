#include <motor/multirotor_quad_x_motor_mapper.hpp>

MultirotorQuadXMotorMapper::MultirotorQuadXMotorMapper() {
}

void MultirotorQuadXMotorMapper::init() {
  PWMMotorMapper::init();
}

void MultirotorQuadXMotorMapper::run(actuator_setpoint_t& input) {
  float outputs[4] = {
     1.0f * input.pitch_sp + 1.0f * input.roll_sp + 1.0f * input.yaw_sp, // front left
    -1.0f * input.pitch_sp + 1.0f * input.roll_sp - 1.0f * input.yaw_sp, // front right
    -1.0f * input.pitch_sp - 1.0f * input.roll_sp + 1.0f * input.yaw_sp, // back right
     1.0f * input.pitch_sp - 1.0f * input.roll_sp - 1.0f * input.yaw_sp  // back left
  };

  for(int i = 0; i < 4; i++) {
    setMotorSpeed(i, 0.50f); // TODO: Fix this
  }
}
