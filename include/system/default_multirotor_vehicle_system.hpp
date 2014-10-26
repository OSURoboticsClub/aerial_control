#ifndef DEFAULT_MULTIROTOR_VEHICLE_SYSTEM_HPP_
#define DEFAULT_MULTIROTOR_VEHICLE_SYSTEM_HPP_

#include <motor/multirotor_quad_x_motor_mapper.hpp>
#include <system/multirotor_vehicle_system.hpp>

class DefaultMultirotorVehicleSystem : public MultirotorVehicleSystem<4> {
public:
  DefaultMultirotorVehicleSystem(Accelerometer *accelerometer, Gyroscope *gyroscope);

  void init();
  void update();

protected:
  Accelerometer *getAccelerometer();
  Gyroscope *getGyroscope();
  AttitudeEstimator *getAttitudeEstimator();
  InputSource *getInputSource();
  ControllerPipeline *getPipeline();
  MotorMapper *getMotorMapper();

private:
  Accelerometer *accelerometer;
  Gyroscope *gyroscope;

  DCMAttitudeEstimator estimator;
  PWMReceiverInputSource inputSource;

  AttitudeController attController;
  AttitudeRateController attRateController;
  Controller *controllers[2];
  ControllerPipeline pipeline;

  MultirotorQuadXMotorMapper motorMapper;
};

#include <system/default_multirotor_vehicle_system.tpp>

#endif
