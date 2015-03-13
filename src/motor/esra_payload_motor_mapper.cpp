#include "motor/esra_payload_motor_mapper.hpp"

#include <array>
#include <cstddef>

#include "protocol/messages.hpp"

EsraPayloadMotorMapper::EsraPayloadMotorMapper(PWMDeviceGroup<1>& motors, Communicator& communicator)
  : motors(motors),
    throttleStream(communicator, 5) {
}

void EsraPayloadMotorMapper::run(bool armed, ActuatorSetpoint& input) {
  // We only have one motor for now, but just in case..
  std::array<float, 1> outputs;
  outputs[0] = input.throttle;
  motors.set(armed, outputs);

  if(throttleStream.ready()) {
    protocol::message::motor_throttle_message_t msg;
    msg.throttles[0] = outputs[0];
    msg.throttles[1] = 0;
    msg.throttles[2] = 0;
    msg.throttles[3] = 0;
    throttleStream.publish(msg);
  }
}
