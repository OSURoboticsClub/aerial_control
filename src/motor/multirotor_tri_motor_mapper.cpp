#include "motor/multirotor_tri_motor_mapper.hpp"

#include <array>
#include <cstddef>
#include "protocol/messages.hpp"

MultirotorTriMotorMapper::MultirotorTriMotorMapper(PWMDeviceGroup<3>& motors, PWMDeviceGroup<1>& servos, Communicator& communicator)
  : motors(motors),
    servos(servos),
    throttleStream(communicator, 5) {
}

void MultirotorTriMotorMapper::run(bool armed, ActuatorSetpoint& input) {
  // Calculate output shifts
  // TODO(yoos): comment on motor indexing convention starting from positive
  // X in counterclockwise order.
  std::array<float, 3> shifts {
    - 1.0f * input.pitch + 1.0f * input.yaw,   // left
      1.0f * input.roll  - 1.0f * input.yaw,   // tail
      1.0f * input.pitch + 1.0f * input.yaw,   // right
  };

  // Add throttle to shifts to get absolute output value
  std::array<float, 3> outputs;
  for(std::size_t i = 0; i < 4; i++) {
    outputs[i] = input.throttle + shifts[i];
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
