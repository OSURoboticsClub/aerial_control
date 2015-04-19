#ifndef UNIT_DATA_HPP_
#define UNIT_DATA_HPP_

#include "communication/communicator.hpp"
#include "estimator/world_estimator.hpp"
#include "estimator/atmospheric_location_estimator.hpp"
#include "estimator/dcm_attitude_estimator.hpp"
#include "motor/esra_rocket_motor_mapper.hpp"
#include "input/offboard_input_source.hpp"
#include "sensor/sensor_measurements.hpp"
#include "system/rocket_system.hpp"
#include "util/optional.hpp"
#include "variant/platform.hpp"

static const float MOTOR_PWM_MIN = 0.53f;
static const float MOTOR_PWM_MAX = 0.93f;
static const float MOTOR_PWM_SAFE = 0.30f;

struct UnitData {
  PWMDeviceGroup<1> motors;
  EsraRocketMotorMapper motorMapper;

  WorldEstimator world;
  AtmosphericLocationEstimator location;
  DCMAttitudeEstimator attitude;
  OffboardInputSource inputSource;

  RocketSystem system;

  UnitData(Platform& platform, Communicator& communicator)
    : motors(platform.get<PWMPlatform>(),
        { 7 },                                       // channels
        { 0.0f },                                    // offsets
        0.0f, 1.0f,                                  // input range
        MOTOR_PWM_MIN, MOTOR_PWM_MAX, MOTOR_PWM_SAFE // output range
      ),
      motorMapper(motors, communicator),
      location(communicator),
      attitude(communicator),
      world(location, attitude, communicator),
      inputSource(communicator),
      system(
          platform.getIdx<Accelerometer>(0),
          std::experimental::make_optional(&platform.getIdx<Accelerometer>(1)),
          std::experimental::make_optional(&platform.get<Barometer>()),
          std::experimental::make_optional(&platform.get<GPS>()),
          platform.get<Gyroscope>(),
          std::experimental::make_optional(&platform.get<Magnetometer>()),
          world, inputSource, motorMapper, communicator,
          platform) {
  }
};

#endif
