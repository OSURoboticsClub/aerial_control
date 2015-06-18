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
  // TODO(yoos): comment on motor indexing convention starting from positive
  // X in counterclockwise order.
  // Calculate motor outputs
  std::array<float, 3> mOutputs;
  mOutputs[0] = input.throttle + input.roll - input.pitch;   // Left
  mOutputs[1] = input.throttle              + input.pitch;   // Tail
  mOutputs[2] = input.throttle - input.roll - input.pitch;   // Right
  motors.set(armed, mOutputs);

  // Calculate servo output
  std::array<float, 1> sOutputs;
  sOutputs[0] = 0.5 + input.yaw;
  servos.set(armed, sOutputs);

  // DEBUG
  static int loop=0;
  if (loop == 0) {
    chprintf((BaseSequentialStream*)&SD4, "MM %f %f %f %f\r\n", mOutputs[0], mOutputs[1], mOutputs[2], sOutputs[0]);
  }
  loop = (loop+1) % 50;

  protocol::message::motor_throttle_message_t m {
    .time = ST2MS(chibios_rt::System::getTime()),
    .throttles = { mOutputs[0], mOutputs[1], mOutputs[2], sOutputs[3] }
  };

  if(throttleStream.ready()) {
    throttleStream.publish(m);
  }
  logger.write(m);
}
