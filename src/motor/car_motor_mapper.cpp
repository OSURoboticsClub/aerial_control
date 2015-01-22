#include "motor/car_motor_mapper.hpp"

#include <array>
#include <cstddef>
#include "protocol/messages.hpp"

CarMotorMapper::CarMotorMapper(PWMDeviceGroup<4>& motorDevices, PWMDeviceGroup<4>& servoDevices, Communicator& communicator)
  : motorDevices(motorDevices),
    servoDevices(servoDevices),
    throttleStream(communicator, 1) {
}

void CarMotorMapper::run(bool armed, actuator_setpoint_t& input) {
  // Motor speeds depend only on throttle input
  std::array<float, 4> motors {
    input.throttle_sp,   // front left
    input.throttle_sp,   // back left
    input.throttle_sp,   // back right
    input.throttle_sp    // front right
  };

  motorDevices.set(armed, motors);

  // Servos control yaw angle of the vehicle
  std::array<float, 4> servos {
    // TODO(kyle): signs
    -input.yaw_sp / 2,   // front left
    input.yaw_sp / 2,   // back left
    input.yaw_sp / 2,   // back right
    -input.yaw_sp / 2    // front right
  };

  servoDevices.set(armed, servos);

  if(throttleStream.ready()) {
    protocol::message::motor_throttle_message_t msg;

    for(std::size_t i = 0; i < 4; i++) {
      msg.throttles[i] = motors[i];
    }

    throttleStream.publish(msg);
  }
}
