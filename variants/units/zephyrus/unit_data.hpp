#ifndef UNIT_DATA_HPP_
#define UNIT_DATA_HPP_

#include "global_parameters.hpp"
#include "heartbeat_thread.hpp"
#include "unit_config.hpp"
#include "communication/communicator.hpp"
#include "controller/angular_acceleration_controller.hpp"
#include "controller/angular_position_controller.hpp"
#include "controller/angular_velocity_controller.hpp"
#include "estimator/world_estimator.hpp"
#include "estimator/atmospheric_location_estimator.hpp"
#include "estimator/dcm_attitude_estimator.hpp"
#include "filesystem/logger.hpp"
#include "input/ppm_input_source.hpp"
#include "motor/multirotor_tri_motor_mapper.hpp"
#include "params/parameter_repository.hpp"
#include "sensor/sensors.hpp"
#include "sensor/sensor_measurements.hpp"
#include "system/multirotor_vehicle_system.hpp"
#include "util/math.hpp"
#include "util/optional.hpp"
#include "variant/platform.hpp"

static const float MOTOR_PWM_MIN = 0.50f;
static const float MOTOR_PWM_MAX = 0.93f;
static const float MOTOR_PWM_SAFE = 0.30f;

// TODO(yoos): Use actual values.
static const float SERVO_PWM_MIN = 0.033f;
static const float SERVO_PWM_MAX = 0.083f;
static const float SERVO_PWM_SAFE = 0.058f;

struct UnitData {
  PWMDeviceGroup<3> motors;
  PWMDeviceGroup<1> servos;
  MultirotorTriMotorMapper motorMapper;

  AtmosphericLocationEstimator location;
  DCMAttitudeEstimator attitude;
  WorldEstimator world;

  PPMInputSourceConfig ppmConfig;
  PPMInputSource inputSource;

  Sensors sensors;
  MultirotorVehicleSystem system;

  UnitData(Platform& platform, ParameterRepository& params, Communicator& communicator, Logger& logger)
    : motors(platform.get<PWMPlatform>(),
        { 0, 1, 2 },                                 // channels
        { 0.0f, 0.0f, 0.0f },                        // offsets
        0.0f, 1.0f,                                  // input range
        MOTOR_PWM_MIN, MOTOR_PWM_MAX, MOTOR_PWM_SAFE // output range
      ),
      servos(platform.get<PWMPlatform>(),
        { 7 },                                       // channels
        { 0.0f },                                    // offsets
        0.0f, 1.0f,                                  // input range
        SERVO_PWM_MIN, SERVO_PWM_MAX, SERVO_PWM_SAFE // output range
      ),
      motorMapper(motors, servos, communicator, logger),
      location(communicator, logger),
      attitude(params, communicator, logger),
      world(location, attitude, communicator, logger),
      ppmConfig{
        .minStartWidth   = 2500,
        .minChannelWidth = 950,
        .maxChannelWidth = 2050,
        .channelThrottle = 0,
        .channelRoll     = 1,
        .channelPitch    = 2,
        .channelYaw      = 3,
        .channelArmed    = 4,
        .channelVelocityMode = 7,
        .channelRange = 6,
        .channelControlMode = 5
      },
      inputSource(ppmConfig),
      sensors(std::experimental::make_optional(&platform.get<Accelerometer>()),
              std::experimental::nullopt,   // No high-range accelerometer
              std::experimental::make_optional(&platform.get<Gyroscope>()),
              std::experimental::make_optional(&platform.get<Barometer>()),
              std::experimental::make_optional(&platform.get<GPS>()),
              std::experimental::nullopt    // No magnetometer
      ),
      system(params,
             sensors,
             world, inputSource, motorMapper, communicator, logger, platform) {

    params.set(GlobalParameters::PARAM_DT, 0.001);
    params.set(HeartbeatThread::PARAM_BLINK_FREQUENCY, 5);

    params.set(AngularPositionController::PARAM_PID_ROLL_KP, 1.2);
    params.set(AngularPositionController::PARAM_PID_ROLL_KI, 0.0);
    params.set(AngularPositionController::PARAM_PID_ROLL_KD, 0.0);
    params.set(AngularPositionController::PARAM_PID_PITCH_KP, 1.2);
    params.set(AngularPositionController::PARAM_PID_PITCH_KI, 0.0);
    params.set(AngularPositionController::PARAM_PID_PITCH_KD, 0.0);
    params.set(AngularPositionController::PARAM_PID_YAW_KP, 1.0);
    params.set(AngularPositionController::PARAM_PID_YAW_KI, 0.0);
    params.set(AngularPositionController::PARAM_PID_YAW_KD, 0.0);
    params.set(AngularPositionController::PARAM_MAX_PITCH_ROLL_POS, M_PI / 3.0);

    params.set(AngularVelocityController::PARAM_PID_ROLL_KP, 0.2);
    params.set(AngularVelocityController::PARAM_PID_ROLL_KI, 0.0);
    params.set(AngularVelocityController::PARAM_PID_ROLL_KD, 0.0);
    params.set(AngularVelocityController::PARAM_PID_PITCH_KP, 0.2);
    params.set(AngularVelocityController::PARAM_PID_PITCH_KI, 0.0);
    params.set(AngularVelocityController::PARAM_PID_PITCH_KD, 0.0);
    params.set(AngularVelocityController::PARAM_PID_YAW_KP, 0.2);
    params.set(AngularVelocityController::PARAM_PID_YAW_KI, 0.0);
    params.set(AngularVelocityController::PARAM_PID_YAW_KD, 0.0);
    params.set(AngularVelocityController::PARAM_MAX_PITCH_ROLL_VEL, 4.0 * M_PI);

    params.set(AngularAccelerationController::PARAM_PID_ROLL_KP, 1.0);
    params.set(AngularAccelerationController::PARAM_PID_ROLL_KI, 0.0);
    params.set(AngularAccelerationController::PARAM_PID_ROLL_KD, 0.0);
    params.set(AngularAccelerationController::PARAM_PID_PITCH_KP, 1.0);
    params.set(AngularAccelerationController::PARAM_PID_PITCH_KI, 0.0);
    params.set(AngularAccelerationController::PARAM_PID_PITCH_KD, 0.0);
    params.set(AngularAccelerationController::PARAM_PID_YAW_KP, 1.0);
    params.set(AngularAccelerationController::PARAM_PID_YAW_KI, 0.0);
    params.set(AngularAccelerationController::PARAM_PID_YAW_KD, 0.0);
    params.set(AngularAccelerationController::PARAM_MAX_PITCH_ROLL_ACC, 100.0);

    Accelerometer& accelerometer = platform.get<Accelerometer>();
    accelerometer.setAxisConfig(unit_config::ACC_AXES);
    accelerometer.setOffsets(unit_config::ACC_OFFSETS);

    Gyroscope& gyroscope = platform.get<Gyroscope>();
    gyroscope.setAxisConfig(unit_config::GYR_AXES);
    gyroscope.setOffsets(unit_config::GYR_OFFSETS);
  }
};

#endif
