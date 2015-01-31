#ifndef ROCKET_SYSTEM_HPP_
#define ROCKET_SYSTEM_HPP_

#include "communication/communicator.hpp"
#include "communication/message_listener.hpp"
#include "controller/angular_position_controller.hpp"
#include "controller/angular_velocity_controller.hpp"
#include "controller/rocket_angular_acceleration_controller.hpp"
#include "controller/position_controller.hpp"
#include "controller/controller_pipeline.hpp"
#include "controller/setpoint_types.hpp"
#include "controller/zero_controller.hpp"
#include "estimator/attitude_estimator.hpp"
#include "input/input_source.hpp"
#include "motor/motor_mapper.hpp"
#include "motor/pwm_device_group.hpp"
#include "sensor/gyroscope.hpp"
#include "sensor/accelerometer.hpp"
#include "system/vehicle_system.hpp"

enum class RocketStage {
  DISABLED,
  PAD,
  ASCENT,
  DESCENT
};

class RocketSystem : public VehicleSystem, public MessageListener {
public:
  RocketSystem(Gyroscope& gyroscope, Accelerometer& accelerometer,
      AttitudeEstimator& estimator, InputSource& inputSource,
      MotorMapper& motorMapper, Communicator& communicator);

  void update() override;

  void on(const protocol::message::set_arm_state_message_t& m) override;

private:
  Gyroscope& gyroscope;
  Accelerometer& accelerometer;

  AttitudeEstimator& estimator;
  InputSource& inputSource;

  PositionController posController;
  AngularPositionController attPosController;
  AngularVelocityController attVelController;
  RocketAngularAccelerationController attAccController;
  ControllerPipeline<actuator_setpoint_t> pipeline;

  ZeroController<actuator_setpoint_t> zeroController;

  MotorMapper& motorMapper;

  RocketStage stage;
};

#endif
