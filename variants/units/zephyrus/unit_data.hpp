#ifndef UNIT_DATA_HPP_
#define UNIT_DATA_HPP_

#include "communication/communicator.hpp"
#include "estimator/world_estimator.hpp"
#include "estimator/atmospheric_location_estimator.hpp"
#include "estimator/dcm_attitude_estimator.hpp"
#include "filesystem/logger.hpp"
#include "input/offboard_input_source.hpp"
#include "motor/multirotor_tri_motor_mapper.hpp"
#include "sensor/sensor_measurements.hpp"
#include "system/multirotor_vehicle_system.hpp"
#include "util/optional.hpp"
#include "variant/platform.hpp"

static const float MOTOR_PWM_MIN = 0.53f;
static const float MOTOR_PWM_MAX = 0.93f;
static const float MOTOR_PWM_SAFE = 0.30f;

// TODO(yoos): Use actual values.
static const float SERVO_PWM_MIN = 0.53f;
static const float SERVO_PWM_MAX = 0.93f;
static const float SERVO_PWM_SAFE = 0.30f;

struct UnitData {
  PWMDeviceGroup<3> motors;
  PWMDeviceGroup<1> servos;
  MultirotorTriMotorMapper motorMapper;

  WorldEstimator world;
  AtmosphericLocationEstimator location;
  DCMAttitudeEstimator attitude;
  OffboardInputSource inputSource;

  MultirotorVehicleSystem system;

  UnitData(Platform& platform, Communicator& communicator, Logger& logger)
    : motors(platform.get<PWMPlatform>(),
        { 0, 1, 2 },                                 // channels
        { 0.0f, 0.0f, 0.0f },                        // offsets
        0.0f, 1.0f,                                  // input range
        MOTOR_PWM_MIN, MOTOR_PWM_MAX, MOTOR_PWM_SAFE // output range
      ),
      servos(platform.get<PWMPlatform>(),
        { 4 },                                       // channels
        { 0.0f },                                    // offsets
        0.0f, 1.0f,                                  // input range
        SERVO_PWM_MIN, SERVO_PWM_MAX, SERVO_PWM_SAFE // output range
      ),
      motorMapper(motors, servos, communicator, logger),
      location(communicator, logger),
      attitude(communicator, logger),
      world(location, attitude, communicator, logger),
      inputSource(communicator),
      system(platform.get<Gyroscope>(), platform.get<Accelerometer>(),
             std::experimental::make_optional(&platform.get<Barometer>()),
             std::experimental::make_optional(&platform.get<GPS>()),
             std::experimental::nullopt,   // No magnetometer
             world, inputSource, motorMapper, communicator) {
  }
};

#endif
