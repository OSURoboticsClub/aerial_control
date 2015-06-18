#include "motor/multirotor_tri_motor_mapper.hpp"

#include <array>
#include <cstddef>
#include "protocol/messages.hpp"
#include "util/time.hpp"
#include <chprintf.h>

MultirotorTriMotorMapper::MultirotorTriMotorMapper(PWMDeviceGroup<3>& motors, PWMDeviceGroup<1>& servos, Communicator& communicator, Logger& logger)
  : motors(motors),
    servos(servos),
    throttleStream(communicator, 5),
    logger(logger){
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

  // DEBUG
  static int loop=0;
  if (loop == 0) {
    chprintf((BaseSequentialStream*)&SD4, "MM %f %f %f %f\r\n", outputs[0], outputs[1], outputs[2], outputs[3]);
  }
  loop = (loop+1) % 50;

  protocol::message::motor_throttle_message_t m {
    .time = ST2MS(chibios_rt::System::getTime()),
    .throttles = { outputs[0], outputs[1], outputs[2], outputs[3] }
  };

  if(throttleStream.ready()) {
    throttleStream.publish(m);
  }
  logger.write(m);
}
