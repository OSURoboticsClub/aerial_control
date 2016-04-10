#ifndef UNIT_DATA_HPP_
#define UNIT_DATA_HPP_

#include "communication/communicator.hpp"
#include "controller/rocket_angular_acceleration_controller.hpp"
#include "estimator/world_estimator.hpp"
#include "estimator/atmospheric_location_estimator.hpp"
#include "estimator/dcm_attitude_estimator.hpp"
#include "filesystem/logger.hpp"
#include "input/offboard_input_source.hpp"
#include "motor/esra_rocket_motor_mapper.hpp"
#include "sensor/sensor_measurements.hpp"
#include "system/canard_system.hpp"
#include "util/optional.hpp"
#include "variant/platform.hpp"

static const float MOTOR_PWM_MIN = 0.300f;
static const float MOTOR_PWM_MAX = 0.440f;
static const float MOTOR_PWM_SAFE = 0.370f;

struct UnitData {
  PWMDeviceGroup<1> motors;
  EsraRocketMotorMapper motorMapper;

  WorldEstimator world;
  AtmosphericLocationEstimator location;
  DCMAttitudeEstimator attitude;
  OffboardInputSource inputSource;

  CanardSystem system;

  UnitData(Platform& platform, ParameterRepository &params, Communicator& communicator, Logger& logger)
    : motors(platform.get<PWMPlatform>(),
        { 7 },                                       // channels
        { 0.0f },                                    // offsets
        0.0f, 1.0f,                                  // input range
        MOTOR_PWM_MIN, MOTOR_PWM_MAX, MOTOR_PWM_SAFE // output range
      ),
      motorMapper(motors, communicator, logger),
      location(communicator, logger),
      attitude(params, communicator, logger),
      world(location, attitude, communicator, logger),
      inputSource(communicator),
      system(
          params,
          platform.getIdx<Accelerometer>(0),
          std::experimental::make_optional(&platform.getIdx<Accelerometer>(1)),
          std::experimental::make_optional(&platform.get<Barometer>()),
          std::experimental::nullopt,
          std::experimental::make_optional(&platform.get<GPS>()),
          platform.get<Gyroscope>(),
          std::experimental::nullopt,
          world, inputSource, motorMapper, communicator, logger,
          platform) {

    params.set(RocketAngularAccelerationController::PARAM_MAX_PITCH_ROLL_ACC, 100.0 * M_PI / 180.0);
  }
};

#endif
