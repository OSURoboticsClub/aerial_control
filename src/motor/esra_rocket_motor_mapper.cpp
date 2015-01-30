#include "motor/esra_rocket_motor_mapper.hpp"

#include <array>
#include <cstddef>
#include <cmath>

EsraRocketMotorMapper::EsraRocketMotorMapper(PWMDeviceGroup<1>& servos, Communicator& communicator)
  : servos(servos),
    throttleStream(communicator, 10) {
}

void EsraRocketMotorMapper::run(bool armed, actuator_setpoint_t& input) {
  // Interpret input roll_sp as servo angle
  std::array<float, 1> outputs { input.roll_sp };
  servos.set(armed, outputs);
}
