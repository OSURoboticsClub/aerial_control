DefaultMultirotorVehicleSystem::DefaultMultirotorVehicleSystem(Accelerometer *accelerometer, Gyroscope *gyroscope)
  : accelerometer(accelerometer), gyroscope(gyroscope) {
}

void DefaultMultirotorVehicleSystem::init() {
  MultirotorVehicleSystem::init();

  motorMapper.init();
}

void DefaultMultirotorVehicleSystem::update() {
  MultirotorVehicleSystem::update();
}

Accelerometer *DefaultMultirotorVehicleSystem::getAccelerometer() {
  return accelerometer;
}

Gyroscope *DefaultMultirotorVehicleSystem::getGyroscope() {
  return gyroscope;
}

AttitudeEstimator *DefaultMultirotorVehicleSystem::getAttitudeEstimator() {
  return &estimator;
}

InputSource *DefaultMultirotorVehicleSystem::getInputSource() {
  return &inputSource;
}

MotorMapper *DefaultMultirotorVehicleSystem::getMotorMapper() {
  return &motorMapper;
}

actuator_setpoint_t DefaultMultirotorVehicleSystem::runController(attitude_estimate_t &estimate, attitude_position_setpoint_t& setpoint) {
  return pipeline.run(estimate, setpoint, attPosController, attVelController);
}
