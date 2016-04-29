#ifndef UNIT_DATA_HPP_
#define UNIT_DATA_HPP_

#include "heartbeat_thread.hpp"
#include "unit_config.hpp"
#include "communication/communicator.hpp"
#include "controller/angular_position_controller.hpp"
#include "controller/angular_velocity_controller.hpp"
#include "controller/rocket_angular_acceleration_controller.hpp"
#include "estimator/world_estimator.hpp"
#include "estimator/atmospheric_location_estimator.hpp"
#include "estimator/dcm_attitude_estimator.hpp"
#include "filesystem/logger.hpp"
#include "input/offboard_input_source.hpp"
#include "motor/esra_rocket_motor_mapper.hpp"
#include "sensor/sensors.hpp"
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

  Sensors sensors;
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
      sensors(std::experimental::make_optional(&platform.get<Accelerometer>()),
              std::experimental::nullopt,   // No high-range accelerometer
              std::experimental::make_optional(&platform.get<Gyroscope>()),
              std::experimental::make_optional(&platform.get<Barometer>()),
              std::experimental::make_optional(&platform.get<GPS>()),
              std::experimental::nullopt   // No magnetometer
      ),
      system(params,
             sensors,
             world, inputSource, motorMapper, communicator, logger, platform) {

    params.set(HeartbeatThread::PARAM_BLINK_FREQUENCY, 3);

    params.set(AngularPositionController::PARAM_PID_ROLL_KP, 1.0);
    params.set(AngularPositionController::PARAM_PID_ROLL_KI, 0.0);
    params.set(AngularPositionController::PARAM_PID_ROLL_KD, 0.0);
    params.set(AngularPositionController::PARAM_PID_PITCH_KP, 1.0);
    params.set(AngularPositionController::PARAM_PID_PITCH_KI, 0.0);
    params.set(AngularPositionController::PARAM_PID_PITCH_KD, 0.0);
    params.set(AngularPositionController::PARAM_PID_YAW_KP, 1.0);
    params.set(AngularPositionController::PARAM_PID_YAW_KI, 0.0);
    params.set(AngularPositionController::PARAM_PID_YAW_KD, 0.0);
    //params.set(AngularPositionController::PARAM_MAX_PITCH_ROLL_POS, M_PI / 3.0);

    params.set(AngularVelocityController::PARAM_PID_ROLL_KP, 1.0);
    params.set(AngularVelocityController::PARAM_PID_ROLL_KI, 0.0);
    params.set(AngularVelocityController::PARAM_PID_ROLL_KD, 0.0);
    params.set(AngularVelocityController::PARAM_PID_PITCH_KP, 1.0);
    params.set(AngularVelocityController::PARAM_PID_PITCH_KI, 0.0);
    params.set(AngularVelocityController::PARAM_PID_PITCH_KD, 0.0);
    params.set(AngularVelocityController::PARAM_PID_YAW_KP, 1.0);
    params.set(AngularVelocityController::PARAM_PID_YAW_KI, 0.0);
    params.set(AngularVelocityController::PARAM_PID_YAW_KD, 0.0);
    //params.set(AngularVelocityController::PARAM_MAX_PITCH_ROLL_VEL, 4.0 * M_PI);

    params.set(RocketAngularAccelerationController::PARAM_MAX_PITCH_ROLL_ACC, 100.0 * M_PI / 180.0);

    Accelerometer& accelerometer = platform.get<Accelerometer>();
    accelerometer.setAxisConfig(unit_config::ACC_AXES);
    accelerometer.setOffsets(unit_config::ACC_OFFSETS);

    Gyroscope& gyroscope = platform.get<Gyroscope>();
    gyroscope.setAxisConfig(unit_config::GYR_AXES);
    gyroscope.setOffsets(unit_config::GYR_OFFSETS);
  }
};

#endif
