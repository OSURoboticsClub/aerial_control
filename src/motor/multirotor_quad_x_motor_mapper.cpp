#include "motor/multirotor_quad_x_motor_mapper.hpp"

#include <array>
#include <cstddef>

MultirotorQuadXMotorMapper::MultirotorQuadXMotorMapper(PWMPlatform& pwmPlatform)
  : PWMMotorMapper(pwmPlatform) {
}

void MultirotorQuadXMotorMapper::init() {
  PWMMotorMapper::init();
}

void MultirotorQuadXMotorMapper::run(actuator_setpoint_t& input) {
  // Calculate output shifts
  // TODO(yoos): comment on motor indexing convention starting from positive
  // X in counterclockwise order.
  std::array<float, 4> output_shifts = {
      1.0f * input.roll_sp - 1.0f * input.pitch_sp + 1.0f * input.yaw_sp,   // front left
      1.0f * input.roll_sp + 1.0f * input.pitch_sp - 1.0f * input.yaw_sp,   // back left
    - 1.0f * input.roll_sp + 1.0f * input.pitch_sp + 1.0f * input.yaw_sp,   // back right
    - 1.0f * input.roll_sp - 1.0f * input.pitch_sp - 1.0f * input.yaw_sp    // front right
  };

  // Add throttle to shifts to get absolute output value
  std::array<float, 4> outputs = { 0 };
  for(std::size_t i = 0; i < 4; i++) {
    outputs[i] = input.throttle_sp + output_shifts[i];
  }

  setMotorSpeeds(outputs);
}
