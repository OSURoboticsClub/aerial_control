#ifndef UNIT_DATA_HPP_
#define UNIT_DATA_HPP_

#include "sensor/gyroscope.hpp"
#include "sensor/accelerometer.hpp"
#include "system/default_multirotor_vehicle_system.hpp"
#include "variant/pwm_platform.hpp"

struct unit_data_t {
  DefaultMultirotorVehicleSystem system;

  unit_data_t(Gyroscope& gyro, Accelerometer& accel, PWMPlatform& pwmPlatform)
    : system(gyro, accel, pwmPlatform) {
  }
};

#endif
