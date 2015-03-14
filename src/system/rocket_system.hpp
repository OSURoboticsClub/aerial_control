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

// Platform
#include "variant/digital_platform.hpp"
#include "variant/pwm_platform.hpp"
#include "variant/platform.hpp"

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
      MotorMapper& motorMapper, Communicator& communicator,
      Platform& platform);

  void update() override;
  bool healthy();

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
  Platform& platform;

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
   *
   * Detect apogee based mainly on four measurements:
   *
   *   1. Pressure rate of change
   *   2. Net acceleration magnitude
   *   3. Non-roll rotation rate magnitude
   *   4. Orientation
   *
   * We first determine possible apogee based on the altitude rate of change,
   * which is estimated from the barometer and GPS. (Currently, only the
   * barometer is used for altitude.)
   *
   * We check for the motor cutoff acceleration drop to negative. If we sense
   * we are falling faster than -40 m/s, we transition to APOGEE. Otherwise, we
   * check for the ideal case of zero altitude change and that we are not
   * observing a false apogee during a subsonic transition.
   *
   * We do not track powered and coasting portions of flight in separate states
   * because a mach transition may happen at an unknowable time, and it's
   * probably easier to track state variables this way.
   */
  RocketState FlightState(SensorMeasurements meas, WorldEstimate est);

  /**
   * Deploy drogue chute. If magnitude of net proper acceleration does not
   * change within 5 seconds, deploy main.
   */
  RocketState ApogeeState(SensorMeasurements meas, WorldEstimate est);

  /**
   * Deploy main chute at 1500' AGL.
   */
  RocketState DescentState(SensorMeasurements meas, WorldEstimate est);

  /**
   * Turn off all telemetry (maybe?) except GPS.
   *
   * Try to conserve power.
   */
  RocketState RecoveryState(SensorMeasurements meas, WorldEstimate est);

  /**
   * RGB LED stuff.
   */
  void SetLED(float r, float g, float b);
  void BlinkLED(float r, float g, float b, float freq);
  void PulseLED(float r, float g, float b, float freq);
  void RGBLED(float freq);

  /**
   * Per-launch calibration
   */
  float groundAltitude;
  float velocity;
};

#endif
