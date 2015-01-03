#include "system/esra_rocket_system.hpp"

EsraRocketSystem::EsraRocketSystem(Gyroscope& gyroscope, Accelerometer& accelerometer, PWMPlatform& pwmPlatform)
  : gyroscope(gyroscope), accelerometer(accelerometer), motorMapper(pwmPlatform) {
}

void EsraRocketSystem::init() {
  RocketSystem::init();

  motorMapper.init();
}

void EsraRocketSystem::update() {
  RocketSystem::update();
}

Gyroscope& EsraRocketSystem::getGyroscope() {
  return gyroscope;
}

Accelerometer& EsraRocketSystem::getAccelerometer() {
  return accelerometer;
}

AttitudeEstimator& EsraRocketSystem::getAttitudeEstimator() {
  return estimator;
}

InputSource& EsraRocketSystem::getInputSource() {
  return inputSource;
}

MotorMapper& EsraRocketSystem::getMotorMapper() {
  return motorMapper;
}

actuator_setpoint_t EsraRocketSystem::runController(const attitude_estimate_t& estimate, const angular_position_setpoint_t& setpoint) {
  return pipeline.run(estimate, setpoint, attPosController, attVelController);
}
