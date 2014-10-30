#ifndef DEFAULT_MULTIROTOR_VEHICLE_SYSTEM_HPP_
#define DEFAULT_MULTIROTOR_VEHICLE_SYSTEM_HPP_

#include <controller/attitude_position_controller.hpp>
#include <controller/attitude_velocity_controller.hpp>
#include <controller/controller_pipeline.hpp>
#include <estimator/attitude_estimator.hpp>
#include <estimator/dcm_attitude_estimator.hpp>
#include <input/input_source.hpp>
#include <input/pwm_receiver_input_source.hpp>
#include <motor/multirotor_quad_x_motor_mapper.hpp>
#include <sensor/accelerometer.hpp>
#include <sensor/gyroscope.hpp>
#include <system/multirotor_vehicle_system.hpp>

class DefaultMultirotorVehicleSystem : public MultirotorVehicleSystem<4> {
public:
  DefaultMultirotorVehicleSystem(Accelerometer *accelerometer, Gyroscope *gyroscope);

  void init() override;
  void update() override;

protected:
  Accelerometer *getAccelerometer() override;
  Gyroscope *getGyroscope() override;
  AttitudeEstimator *getAttitudeEstimator() override;
  InputSource *getInputSource() override;
  MotorMapper *getMotorMapper() override;

  actuator_setpoint_t runController(attitude_estimate_t &estimate, attitude_position_setpoint_t& setpoint) override;

private:
  Accelerometer *accelerometer;
  Gyroscope *gyroscope;

  DCMAttitudeEstimator estimator;
  PWMReceiverInputSource inputSource;

  AttitudePositionController attPosController;
  AttitudeVelocityController attVelController;
  ControllerPipeline<actuator_setpoint_t> pipeline;

  MultirotorQuadXMotorMapper motorMapper;
};

#include <system/default_multirotor_vehicle_system.tpp>

#endif
