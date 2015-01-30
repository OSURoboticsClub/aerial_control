#include "motor/multirotor_quad_plus_motor_mapper.hpp"

#include <array>
#include <cstddef>

#include "protocol/messages.hpp"

MultirotorQuadPlusMotorMapper::MultirotorQuadPlusMotorMapper(PWMDeviceGroup<4>& motors, Communicator& communicator)
  : motors(motors),
    throttleStream(communicator, 10) {
}

void MultirotorQuadPlusMotorMapper::run(bool armed, actuator_setpoint_t& input) {
  // Calculate output shifts
  // TODO(yoos): comment on motor indexing convention starting from positive
  // X in counterclockwise order.
  std::array<float, 4> shifts {
    - 1.0f * input.pitch_sp + 1.0f * input.yaw_sp,   // front
      1.0f * input.roll_sp  - 1.0f * input.yaw_sp,   // left
      1.0f * input.pitch_sp + 1.0f * input.yaw_sp,   // back
    - 1.0f * input.roll_sp  - 1.0f * input.yaw_sp    // right
  };

  // Add throttle to shifts to get absolute output value
  std::array<float, 4> outputs;
  for(std::size_t i = 0; i < 4; i++) {
    outputs[i] = input.throttle_sp + shifts[i];
  }

  motors.set(armed, outputs);

  if(throttleStream.ready()) {
    protocol::message::motor_throttle_message_t msg;

    for(std::size_t i = 0; i < 4; i++) {
      msg.throttles[i] = outputs[i];
    }

    throttleStream.publish(msg);
  }
}
