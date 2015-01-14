#ifndef MULTIROTOR_SYSTEM_HPP_
#define MULTIROTOR_SYSTEM_HPP_

#include "communication/communicator.hpp"
#include "communication/message_listener.hpp"
#include "controller/angular_position_controller.hpp"
#include "controller/angular_velocity_controller.hpp"
#include "controller/position_controller.hpp"
#include "controller/controller_pipeline.hpp"
#include "controller/setpoint_types.hpp"
#include "controller/zero_controller.hpp"
#include "estimator/attitude_estimator.hpp"
#include "input/input_source.hpp"
#include "motor/motor_mapper.hpp"
#include "sensor/gyroscope.hpp"
#include "sensor/accelerometer.hpp"
#include "system/vehicle_system.hpp"

enum class MultirotorControlMode {
  POSITION,
  VELOCITY,
  ANGULAR_POS,
  ANGULAR_RATE,
};

class MultirotorVehicleSystem : public VehicleSystem, public MessageListener {
public:
  MultirotorVehicleSystem(Communicator& communicator);

  void init() override;
  void update() override;

public:
  virtual Gyroscope& getGyroscope() = 0;
  virtual Accelerometer& getAccelerometer() = 0;
  virtual AttitudeEstimator& getAttitudeEstimator() = 0;
  virtual InputSource& getInputSource() = 0;
  virtual MotorMapper& getMotorMapper() = 0;

  void on(const protocol::message::set_arm_state_message_t& m) override;

private:
  template <typename SP>
  actuator_setpoint_t runPipeline(const attitude_estimate_t& estimate, const SP& sp);

  MultirotorControlMode mode;

  PositionController posController;
  AngularPositionController attPosController;
  AngularVelocityController attVelController;
  ControllerPipeline<actuator_setpoint_t> pipeline;

  ZeroController<actuator_setpoint_t> zeroController;
};

#include "system/multirotor_vehicle_system.tpp"

#endif
