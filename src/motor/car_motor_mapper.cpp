#include "motor/car_motor_mapper.hpp"

#include <array>
#include <cstddef>
#include "protocol/messages.hpp"

CarMotorMapper::CarMotorMapper(PWMPlatform& pwmPlatform, Communicator& communicator)
  : PWMMotorMapper(pwmPlatform),
    throttleStream(communicator, 10) {
}

void CarMotorMapper::init() {
  PWMMotorMapper::init();
}

void CarMotorMapper::run(bool armed, actuator_setpoint_t& input) {
  // Calculate output shifts
  // TODO(yoos): comment on motor indexing convention starting from positive
  // X in counterclockwise order.
  std::array<float, 4> motorOutputs {
    input.throttle_sp,   // front left
    input.throttle_sp,   // back left
    input.throttle_sp,   // back right
    input.throttle_sp    // front right
  };

  setMotorSpeeds(armed, 0, motorOutputs);

  std::array<float, 4> servoOutputs {
    // TODO: signs
    input.yaw_sp,   // front left
    -input.yaw_sp,   // back left
    -input.yaw_sp,   // back right
    input.yaw_sp    // front right
  };

  setMotorSpeeds(armed, 4, servoOutputs, -1.0f, 1.0f);

  if(throttleStream.ready()) {
    protocol::message::motor_throttle_message_t msg;

    for(std::size_t i = 0; i < 4; i++) {
      msg.throttles[i] = motorOutputs[i];
    }

    throttleStream.publish(msg);
  }
}
