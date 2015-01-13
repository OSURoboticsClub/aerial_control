#ifndef DEFAULT_MULTIROTOR_VEHICLE_SYSTEM_HPP_
#define DEFAULT_MULTIROTOR_VEHICLE_SYSTEM_HPP_

#include "communication/communicator.hpp"
#include "controller/angular_position_controller.hpp"
#include "controller/angular_velocity_controller.hpp"
#include "controller/controller_pipeline.hpp"
#include "controller/zero_controller.hpp"
#include "estimator/attitude_estimator.hpp"
#include "estimator/dcm_attitude_estimator.hpp"
#include "input/input_source.hpp"
#include "input/offboard_input_source.hpp"
#include "motor/multirotor_quad_x_motor_mapper.hpp"
#include "sensor/gyroscope.hpp"
#include "sensor/accelerometer.hpp"
#include "system/multirotor_vehicle_system.hpp"

class DefaultMultirotorVehicleSystem : public MultirotorVehicleSystem<4> {
public:
  DefaultMultirotorVehicleSystem(Gyroscope& gyroscope, Accelerometer& accelerometer,
      PWMPlatform& pwmPlatform, Communicator& communicator);

  void init() override;
  void update() override;

protected:
  Gyroscope& getGyroscope() override;
  Accelerometer& getAccelerometer() override;
  AttitudeEstimator& getAttitudeEstimator() override;
  InputSource& getInputSource() override;
  MotorMapper& getMotorMapper() override;

  actuator_setpoint_t runController(const attitude_estimate_t& estimate, const angular_position_setpoint_t& setpoint) override;

private:
  Gyroscope& gyroscope;
  Accelerometer& accelerometer;

  DCMAttitudeEstimator estimator;

  OffboardInputSource inputSource;

  AngularPositionController attPosController;
  AngularVelocityController attVelController;
  ControllerPipeline<actuator_setpoint_t> pipeline;

  MultirotorQuadXMotorMapper motorMapper;
};

#endif
