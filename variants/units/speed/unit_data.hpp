#ifndef UNIT_DATA_HPP_
#define UNIT_DATA_HPP_

#include "communication/communicator.hpp"
#include "sensor/gyroscope.hpp"
#include "sensor/accelerometer.hpp"
#include "system/car_vehicle_system.hpp"
#include "variant/pwm_platform.hpp"

struct unit_data_t {
  CarVehicleSystem system;

  unit_data_t(Gyroscope& gyro, Accelerometer& accel, PWMPlatform& pwmPlatform, Communicator& communicator)
    : system(gyro, accel, pwmPlatform, communicator) {
  }
};

#endif
