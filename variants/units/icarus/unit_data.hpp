#ifndef UNIT_DATA_HPP_
#define UNIT_DATA_HPP_

#include "system/rocket_system.hpp"
#include "motor/esra_rocket_motor_mapper.hpp"
#include "estimator/atmospheric_location_estimator.hpp"
#include "estimator/dcm_attitude_estimator.hpp"
#include "estimator/world_estimator.hpp"
#include "communication/communicator.hpp"
#include "input/offboard_input_source.hpp"
#include "sensor/gyroscope.hpp"
#include "sensor/accelerometer.hpp"
#include "variant/platform.hpp"

static const float MOTOR_PWM_MIN = 0.40f;
static const float MOTOR_PWM_MAX = 0.50f;
static const float MOTOR_PWM_SAFE = 0.0f;   // Disable servo

struct UnitData {
  PWMDeviceGroup<1> servos;
  EsraRocketMotorMapper motorMapper;

  AtmosphericLocationEstimator location;
  DCMAttitudeEstimator attitude;
  WorldEstimator world;
  OffboardInputSource inputSource;

  RocketSystem system;

  UnitData(Gyroscope& gyro, Accelerometer& accel, PWMPlatform& pwmPlatform,
      Communicator& communicator)
    : servos(pwmPlatform,
        { 0 },                              // channels
        { 0.0f },                  // offsets
        0.0f, 1.0f,                                  // input range
        MOTOR_PWM_MIN, MOTOR_PWM_MAX, MOTOR_PWM_SAFE // output range
      ),
      motorMapper(servos, communicator),
      location(communicator),
      attitude(communicator),
      world(location, attitude, communicator),
      inputSource(communicator),
      system(gyro, accel, world, inputSource, motorMapper, communicator) {
  }
};

#endif
