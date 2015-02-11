#ifndef CAR_SYSTEM_HPP_
#define CAR_SYSTEM_HPP_

#include "communication/communicator.hpp"
#include "communication/message_listener.hpp"
#include "controller/angular_velocity_controller.hpp"
#include "controller/angular_acceleration_controller.hpp"
#include "controller/controller_pipeline.hpp"
#include "controller/setpoint_types.hpp"
#include "controller/zero_controller.hpp"
#include "estimator/world_estimator.hpp"
#include "input/offboard_input_source.hpp"
#include "motor/car_motor_mapper.hpp"
#include "motor/pwm_device_group.hpp"
#include "sensor/sensor_measurements.hpp"
#include "system/vehicle_system.hpp"
#include "variant/pwm_platform.hpp"

class CarVehicleSystem : public VehicleSystem, public MessageListener {
public:
  CarVehicleSystem(Gyroscope& gyroscope, Accelerometer& accelerometer,
      PWMDeviceGroup<4>& motorDevices, PWMDeviceGroup<4>& servoDevices,
      WorldEstimator& estimator, Communicator& communicator);

  void update() override;

  void on(const protocol::message::set_arm_state_message_t& m) override;

private:
  Gyroscope& gyroscope;
  Accelerometer& accelerometer;

  WorldEstimator& estimator;
  OffboardInputSource inputSource;
  CarMotorMapper motorMapper;

  AngularVelocityController attVelController;
  AngularAccelerationController attAccController;
  ControllerPipeline<ActuatorSetpoint> pipeline;

  ZeroController<ActuatorSetpoint> zeroController;
};

#endif
