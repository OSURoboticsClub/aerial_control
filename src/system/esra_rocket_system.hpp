#ifndef ESRA_ROCKET_SYSTEM_HPP_
#define ESRA_ROCKET_SYSTEM_HPP_

#include "controller/angular_position_controller.hpp"
#include "controller/angular_velocity_controller.hpp"
#include "controller/controller_pipeline.hpp"
#include "estimator/attitude_estimator.hpp"
#include "estimator/dcm_attitude_estimator.hpp"
#include "input/input_source.hpp"
#include "input/pwm_receiver_input_source.hpp"
#include "motor/esra_rocket_motor_mapper.hpp"   // TODO(yoos)
#include "sensor/gyroscope.hpp"
#include "sensor/accelerometer.hpp"
#include "system/rocket_system.hpp"

class EsraRocketSystem : public RocketSystem<4> {
public:
  EsraRocketSystem(Gyroscope& gyroscope, Accelerometer& accelerometer);

  void init() override;
  void update() override;

protected:
  Gyroscope& getGyroscope() override;
  Accelerometer& getAccelerometer() override;
  AttitudeEstimator& getAttitudeEstimator() override;
  InputSource& getInputSource() override;
  MotorMapper& getMotorMapper() override;

  actuator_setpoint_t runController(attitude_estimate_t &estimate, angular_position_setpoint_t& setpoint) override;

private:
  Gyroscope& gyroscope;
  Accelerometer& accelerometer;

  DCMAttitudeEstimator estimator;
  PWMReceiverInputSource inputSource;

  AngularPositionController attPosController;
  AngularVelocityController attVelController;
  ControllerPipeline<actuator_setpoint_t> pipeline;

  EsraRocketMotorMapper motorMapper;
};

#endif
