#ifndef DEFAULT_MULTIROTOR_VEHICLE_SYSTEM_HPP_
#define DEFAULT_MULTIROTOR_VEHICLE_SYSTEM_HPP_

#include <controller/angular_position_controller.hpp>
#include <controller/angular_velocity_controller.hpp>
#include <controller/controller_pipeline.hpp>
#include <estimator/attitude_estimator.hpp>
#include <estimator/dcm_attitude_estimator.hpp>
#include <input/input_source.hpp>
#include <input/pwm_receiver_input_source.hpp>
#include <motor/multirotor_quad_x_motor_mapper.hpp>
#include <sensor/gyroscope.hpp>
#include <sensor/accelerometer.hpp>
#include <system/multirotor_vehicle_system.hpp>

class DefaultMultirotorVehicleSystem : public MultirotorVehicleSystem<4> {
public:
  inline DefaultMultirotorVehicleSystem(Gyroscope *gyroscope, Accelerometer *accelerometer);

  inline void init() override;
  inline void update() override;

protected:
  inline Gyroscope *getGyroscope() override;
  inline Accelerometer *getAccelerometer() override;
  inline AttitudeEstimator *getAttitudeEstimator() override;
  inline InputSource *getInputSource() override;
  inline MotorMapper *getMotorMapper() override;

  inline actuator_setpoint_t runController(const attitude_estimate_t &estimate, const angular_position_setpoint_t& setpoint) override;

private:
  Gyroscope *gyroscope;
  Accelerometer *accelerometer;

  DCMAttitudeEstimator estimator;
  PWMReceiverInputSource inputSource;

  AngularPositionController attPosController;
  AngularVelocityController attVelController;
  ControllerPipeline<actuator_setpoint_t> pipeline;

  MultirotorQuadXMotorMapper motorMapper;
};

#include <system/default_multirotor_vehicle_system.tpp>

#endif
