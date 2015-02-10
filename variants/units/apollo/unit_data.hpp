#ifndef UNIT_DATA_HPP_
#define UNIT_DATA_HPP_

#include "communication/communicator.hpp"
#include "estimator/world_estimator.hpp"
#include "estimator/dcm_attitude_estimator.hpp"
#include "motor/multirotor_quad_plus_motor_mapper.hpp"
#include "input/offboard_input_source.hpp"
#include "sensor/accelerometer.hpp"
#include "sensor/gyroscope.hpp"
#include "sensor/magnetometer.hpp"
#include "system/multirotor_vehicle_system.hpp"
#include "util/optional.hpp"
#include "variant/platform.hpp"

static const float MOTOR_PWM_MIN = 0.53f;
static const float MOTOR_PWM_MAX = 0.93f;
static const float MOTOR_PWM_SAFE = 0.30f;

struct UnitData {
  PWMDeviceGroup<4> motors;
  MultirotorQuadPlusMotorMapper motorMapper;

  WorldEstimator world;
  DCMAttitudeEstimator attitude;
  OffboardInputSource inputSource;

  MultirotorVehicleSystem system;

  UnitData(Platform& platform, Communicator& communicator)
    : motors(platform.get<PWMPlatform>(),
        { 0, 1, 2, 3 },                              // channels
        { 0.0f, 0.0f, 0.0f, 0.0f },                  // offsets
        0.0f, 1.0f,                                  // input range
        MOTOR_PWM_MIN, MOTOR_PWM_MAX, MOTOR_PWM_SAFE // output range
      ),
      motorMapper(motors, communicator),
      world(communicator),
      attitude(communicator),
      inputSource(communicator),
      system(platform.get<Gyroscope>(),
             platform.get<Accelerometer>(),
             std::experimental::nullopt,
             std::experimental::make_optional(&platform.get<Magnetometer>()),
             world, attitude, inputSource, motorMapper, communicator
      ) {
  }
};

#endif
