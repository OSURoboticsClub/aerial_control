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

enum class RocketStage {
  DISABLED,
  PAD,
  ASCENT,
  DESCENT
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

  RocketStage stage;
};

#endif
