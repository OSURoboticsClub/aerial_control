#ifndef UNIT_DATA_HPP_
#define UNIT_DATA_HPP_

#include "communication/communicator.hpp"
#include "estimator/world_estimator.hpp"
#include "estimator/atmospheric_location_estimator.hpp"
#include "estimator/dcm_attitude_estimator.hpp"
#include "motor/multirotor_quad_x_motor_mapper.hpp"
#include "input/ppm_input_source.hpp"
#include "sensor/accelerometer.hpp"
#include "sensor/gyroscope.hpp"
#include "sensor/magnetometer.hpp"
#include "system/multirotor_vehicle_system.hpp"
#include "util/optional.hpp"
#include "variant/platform.hpp"
#include "variant/icu_platform.hpp"

static const float MOTOR_PWM_MIN = 0.532f;
static const float MOTOR_PWM_MAX = 0.932f;
static const float MOTOR_PWM_SAFE = 0.30f;

struct UnitData {
  PWMDeviceGroup<4> motors;
  MultirotorQuadXMotorMapper motorMapper;

  AtmosphericLocationEstimator location;
  DCMAttitudeEstimator attitude;
  WorldEstimator world;
  PPMInputSourceConfig ppmConfig;
  PPMInputSource inputSource;

  MultirotorVehicleSystem system;

  UnitData(Platform& platform, Communicator& communicator)
    : motors(platform.get<PWMPlatform>(),
        { 0, 1, 2, 3 },                              // channels
        { 0.0f, 0.0f, 0.0f, 0.0f },                  // offsets
        0.0f, 1.0f,                                  // input range
        MOTOR_PWM_MIN, MOTOR_PWM_MAX, MOTOR_PWM_SAFE // output range
      ),
      motorMapper(motors, communicator),
      location(communicator),
      attitude(communicator),
      world(location, attitude, communicator),
      ppmConfig{
        .minStartWidth   = 2500,
        .minChannelWidth = 800,
        .maxChannelWidth = 2200,
        .channelThrottle = 0,
        .channelRoll     = 1,
        .channelPitch    = 2,
        .channelYaw      = 3,
        .channelArmed    = 4,
      },
      inputSource(ppmConfig),
      system(platform.get<Gyroscope>(),
             platform.get<Accelerometer>(),
             std::experimental::nullopt,
             std::experimental::make_optional(&platform.get<Magnetometer>()),
             world, inputSource, motorMapper, communicator
      ) {
  }
};

#endif
