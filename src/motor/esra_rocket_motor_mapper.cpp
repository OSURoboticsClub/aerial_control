#include "motor/esra_rocket_motor_mapper.hpp"

#include <array>
#include <cstddef>

EsraRocketMotorMapper::EsraRocketMotorMapper(PWMDeviceGroup<1>& motors, Communicator& communicator)
  : motors(motors),
    throttleStream(communicator, 10) {
}

void EsraRocketMotorMapper::run(bool armed, actuator_setpoint_t& input) {
  std::array<float, 1> outputs { input.throttle_sp };
  motors.set(armed, outputs);
}
