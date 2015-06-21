#ifndef MULTIROTOR_SYSTEM_HPP_
#define MULTIROTOR_SYSTEM_HPP_

#include "system/vehicle_system.hpp"
#include "util/optional.hpp"

// Communication
#include "communication/communicator.hpp"
#include "communication/message_listener.hpp"

// Control
#include "controller/angular_position_controller.hpp"
#include "controller/angular_velocity_controller.hpp"
#include "controller/angular_acceleration_controller.hpp"
#include "controller/position_controller.hpp"
#include "controller/controller_pipeline.hpp"
#include "controller/setpoint_types.hpp"
#include "controller/zero_controller.hpp"
#include "input/input_source.hpp"
#include "motor/motor_mapper.hpp"
#include "motor/pwm_device_group.hpp"

// Filesystem
#include "filesystem/logger.hpp"

// Platform
#include "variant/pwm_platform.hpp"
#include "variant/platform.hpp"

// World estimation
#include "estimator/world_estimator.hpp"

// Sensors
#include "sensor/accelerometer.hpp"
#include "sensor/gps.hpp"
#include "sensor/gyroscope.hpp"
#include "sensor/magnetometer.hpp"

enum class MultirotorControlMode {
  DISARMED,
  POSITION,
  VELOCITY,
  ANGULAR_POS,
  ANGULAR_VEL,
};

class MultirotorVehicleSystem : public VehicleSystem, public MessageListener {
public:
  MultirotorVehicleSystem(
      Gyroscope& gyr,
      Accelerometer& acc,
      optional<Barometer *> bar,
      optional<GPS *> gps,
      optional<Magnetometer *> mag, // TODO: Use reference_wrapper?
      WorldEstimator& estimator, InputSource& inputSource,
      MotorMapper& motorMapper, Communicator& communicator,
      Logger& logger, Platform& platform);

  void update() override;
  bool healthy();

  void on(const protocol::message::set_arm_state_message_t& m) override;

private:
  Gyroscope& gyr;
  Accelerometer& acc;
  optional<Barometer *> bar;
  optional<GPS *> gps;
  optional<Magnetometer *> mag;

  WorldEstimator& estimator;
  InputSource& inputSource;

  PositionController posController;
  AngularPositionController attPosController;
  AngularVelocityController attVelController;
  AngularAccelerationController attAccController;
  ControllerPipeline<ActuatorSetpoint> pipeline;

  ZeroController<ActuatorSetpoint> zeroController;

  MotorMapper& motorMapper;
  Platform& platform;
  RateLimitedStream stream;
  Logger& logger;

  MultirotorControlMode mode;

  bool calibrated;
  void calibrate(SensorMeasurements meas);

  float yawPosSp;

  void DisarmedMode(SensorMeasurements meas, WorldEstimate est, ControllerInput input, ActuatorSetpoint& sp);
  void AngularRateMode(SensorMeasurements meas, WorldEstimate est, ControllerInput input, ActuatorSetpoint& sp);
  void AngularPosMode(SensorMeasurements meas, WorldEstimate est, ControllerInput input, ActuatorSetpoint& sp);
  void PosMode(SensorMeasurements meas, WorldEstimate est, ControllerInput input, ActuatorSetpoint& sp);

  /**
   * RGB LED stuff.
   */
  void SetLED(float r, float g, float b);
  void BlinkLED(float r, float g, float b, float freq);
  void PulseLED(float r, float g, float b, float freq);
  void RGBLED(float freq);
};

#endif
