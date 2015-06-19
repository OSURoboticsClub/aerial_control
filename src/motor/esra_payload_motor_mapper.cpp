#include "motor/esra_payload_motor_mapper.hpp"
#include "util/time.hpp"

#include <array>
#include <cstddef>

#include "protocol/messages.hpp"

EsraPayloadMotorMapper::EsraPayloadMotorMapper(PWMDeviceGroup<1>& motors, Communicator& communicator, Logger& logger)
  : motors(motors),
    throttleStream(communicator, 5),
    logger(logger) {
}

void EsraPayloadMotorMapper::run(bool armed, ActuatorSetpoint& input) {
  // We only have one motor for now, but just in case..
  std::array<float, 1> outputs;
  outputs[0] = input.throttle;
  motors.set(armed, outputs);

  protocol::message::motor_throttle_message_t m {
    .time = ST2MS(chibios_rt::System::getTime()),
    .throttles = { outputs[0], 0, 0, 0 }
  };

  if(throttleStream.ready()) {
    throttleStream.publish(m);
  }
  logger.write(m);
}
