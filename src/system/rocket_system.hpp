#ifndef ROCKET_SYSTEM_HPP_
#define ROCKET_SYSTEM_HPP_

#include "system/vehicle_system.hpp"
#include "util/optional.hpp"

// Communication
#include "communication/communicator.hpp"
#include "communication/message_listener.hpp"

// Control
#include "controller/angular_velocity_controller.hpp"
#include "controller/rocket_angular_acceleration_controller.hpp"
#include "controller/position_controller.hpp"
#include "controller/controller_pipeline.hpp"
#include "controller/setpoint_types.hpp"
#include "controller/zero_controller.hpp"
#include "input/input_source.hpp"
#include "motor/motor_mapper.hpp"
#include "motor/pwm_device_group.hpp"

// World estimation
#include "estimator/world_estimator.hpp"

// Sensors
#include "sensor/sensor_measurements.hpp"

enum class RocketState {
  DISARMED,
  PRE_ARM,
  ARMED,
  FLIGHT,
  APOGEE,
  DESCENT,
  RECOVERY
};

class RocketSystem : public VehicleSystem, public MessageListener {
public:
  RocketSystem(
      Accelerometer& accel,
      optional<Accelerometer *> accelH,
      optional<Barometer *> bar,
      optional<GPS *> gps,
      Gyroscope& gyr,
      optional<Magnetometer *> mag,
      WorldEstimator& estimator, InputSource& inputSource,
      MotorMapper& motorMapper, Communicator& communicator);

  void update() override;

  void on(const protocol::message::set_arm_state_message_t& m) override;

private:
  Accelerometer& accel;
  optional<Accelerometer *> accelH;
  optional<Barometer *> bar;
  optional<GPS *> gps;
  Gyroscope& gyr;
  optional<Magnetometer *> mag;

  WorldEstimator& estimator;
  InputSource& inputSource;

  PositionController posController;
  AngularVelocityController attVelController;
  RocketAngularAccelerationController attAccController;
  ControllerPipeline<ActuatorSetpoint> pipeline;

  ZeroController<ActuatorSetpoint> zeroController;

  MotorMapper& motorMapper;

  /**
   * For now, we proceed directly to PRE_ARM.
   */
  RocketState DisarmedState(SensorMeasurements meas, WorldEstimate est);

  /**
   * Full data stream to ground station begins here.
   * Sensor calibration should be performed, hopefully with physical access to
   * the board.
   *
   * Proceed to ARMED on meeting all of the following conditions:
   *
   *   1. Sensor health checks passed
   *   2. GPS lock
   *   3. Software arm signal received from GS
   */
  RocketState PreArmState(SensorMeasurements meas, WorldEstimate est);

  /**
   * Sensor calibration should be finished.
   *
   * Proceed to FLIGHT if X accel exceeds 1.1g.
   */
  RocketState ArmedState(SensorMeasurements meas, WorldEstimate est);

  /**
   * Begin onboard data logging.
   * NOTE(yoos): Unfortunately, we do not have onboard logging yet.
   */
  RocketState FlightState(SensorMeasurements meas, WorldEstimate est);

  RocketState ApogeeState(SensorMeasurements meas, WorldEstimate est);
  RocketState DescentState(SensorMeasurements meas, WorldEstimate est);
  RocketState RecoveryState(SensorMeasurements meas, WorldEstimate est);
};

#endif
