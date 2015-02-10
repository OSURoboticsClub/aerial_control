#ifndef UNIT_DATA_HPP_
#define UNIT_DATA_HPP_

#include "communication/communicator.hpp"
#include "motor/pwm_device_group.hpp"
#include "sensor/gyroscope.hpp"
#include "sensor/accelerometer.hpp"
#include "system/car_vehicle_system.hpp"
#include "variant/pwm_platform.hpp"

static const float MOTOR_PWM_MIN = 0.53f;
static const float MOTOR_PWM_MAX = 0.93f;
static const float MOTOR_PWM_SAFE = 0.30f;

static const float SERVO_PWM_MIN = 0.005;
static const float SERVO_PWM_MAX = 0.147f;
static const float SERVO_PWM_CENTER = 0.076f;

struct UnitData {
  PWMDeviceGroup<4> motorDevices;
  PWMDeviceGroup<4> servoDevices;
  CarVehicleSystem system;

  UnitData(Gyroscope& gyro, Accelerometer& accel, PWMPlatform& pwmPlatform,
      Communicator& communicator)
    : motorDevices(pwmPlatform, {{ 0, 1, 2, 3 }}, {{ 0.0, 0.0, 0.0, 0.0 }}, 0.0f, 1.0f, MOTOR_PWM_MIN, MOTOR_PWM_MAX, MOTOR_PWM_SAFE),
      servoDevices(pwmPlatform, {{ 4, 5, 6, 7 }}, {{ -0.03, -0.011, -0.011, -0.015 }}, -1.0f, 1.0f, SERVO_PWM_MIN, SERVO_PWM_MAX, SERVO_PWM_CENTER),
      system(gyro, accel, motorDevices, servoDevices, communicator) {
  }
};

#endif
