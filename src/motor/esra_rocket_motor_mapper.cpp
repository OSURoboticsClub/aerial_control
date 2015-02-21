#include "motor/esra_rocket_motor_mapper.hpp"

#include <array>
#include <cstddef>
#include <cmath>

EsraRocketMotorMapper::EsraRocketMotorMapper(PWMDeviceGroup<1>& servos, Communicator& communicator)
  : servos(servos),
    throttleStream(communicator, 5) {
}

void EsraRocketMotorMapper::run(bool armed, ActuatorSetpoint& input) {
  // Interpret input roll as servo angle
  std::array<float, 1> outputs { input.roll };
  servos.set(armed, outputs);
}
