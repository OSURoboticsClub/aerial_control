DefaultMultirotorVehicleSystem::DefaultMultirotorVehicleSystem(Accelerometer *accelerometer, Gyroscope *gyroscope)
  : accelerometer(accelerometer), gyroscope(gyroscope),
    controllers{ &attController, &attRateController },
    pipeline(controllers, 2) {
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

ControllerPipeline *DefaultMultirotorVehicleSystem::getPipeline() {
  return &pipeline;
}

MotorMapper *DefaultMultirotorVehicleSystem::getMotorMapper() {
  return &motorMapper;
}
