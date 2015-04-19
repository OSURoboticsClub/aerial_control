#include "motor/esra_rocket_motor_mapper.hpp"

#include <array>
#include <cstddef>
#include <cmath>

EsraRocketMotorMapper::EsraRocketMotorMapper(PWMDeviceGroup<1>& motors, Communicator& communicator)
  : motors(motors),
    throttleStream(communicator, 5) {
}

void EsraRocketMotorMapper::run(bool armed, ActuatorSetpoint& input) {
  // Interpret input roll as servo angle
  std::array<float, 1> outputs { input.roll };
  motors.set(armed, outputs);
}
