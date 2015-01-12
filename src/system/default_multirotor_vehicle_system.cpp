#include "system/default_multirotor_vehicle_system.hpp"

DefaultMultirotorVehicleSystem::DefaultMultirotorVehicleSystem(Gyroscope& gyroscope, Accelerometer& accelerometer, PWMPlatform& pwmPlatform, Communicator& communicator)
  : MultirotorVehicleSystem(communicator), gyroscope(gyroscope),
    accelerometer(accelerometer), estimator(communicator), motorMapper(pwmPlatform) {
}

void DefaultMultirotorVehicleSystem::init() {
  MultirotorVehicleSystem::init();

  motorMapper.init();
}

void DefaultMultirotorVehicleSystem::update() {
  MultirotorVehicleSystem::update();
}

Gyroscope& DefaultMultirotorVehicleSystem::getGyroscope() {
  return gyroscope;
}

Accelerometer& DefaultMultirotorVehicleSystem::getAccelerometer() {
  return accelerometer;
}

AttitudeEstimator& DefaultMultirotorVehicleSystem::getAttitudeEstimator() {
  return estimator;
}

InputSource& DefaultMultirotorVehicleSystem::getInputSource() {
  return inputSource;
}

MotorMapper& DefaultMultirotorVehicleSystem::getMotorMapper() {
  return motorMapper;
}

actuator_setpoint_t DefaultMultirotorVehicleSystem::runController(const attitude_estimate_t& estimate, const angular_position_setpoint_t& setpoint) {
  return pipeline.run(estimate, setpoint, attPosController, attVelController);
}
