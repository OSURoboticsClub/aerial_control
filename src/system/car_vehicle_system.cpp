#include "system/car_vehicle_system.hpp"

#include "hal.h"

#include "util/optional.hpp"

CarVehicleSystem::CarVehicleSystem(ParameterRepository& params, Gyroscope& gyroscope,
    Accelerometer& accelerometer, PWMDeviceGroup<4>& motorDevices, PWMDeviceGroup<4>& servoDevices,
    Communicator& communicator, Logger& logger)
  : VehicleSystem(communicator), MessageListener(communicator),
    gyroscope(gyroscope), accelerometer(accelerometer),
    locEstimator(communicator, logger), attEstimator(params, communicator, logger),
    worldEstimator(locEstimator, attEstimator, communicator, logger),
    inputSource(communicator), motorMapper(motorDevices, servoDevices, communicator, logger),
    attAccController(params) {
  // Disarm by default. A set_arm_state_message_t message is required to enable
  // the control pipeline.
  setArmed(false);
}

void CarVehicleSystem::update() {
  // Poll the gyroscope and accelerometer
  GyroscopeReading gyroReading = gyroscope.readGyro();
  AccelerometerReading accelReading = accelerometer.readAccel();

  SensorMeasurements meas {
    .accel  = std::experimental::make_optional(accelReading),
    .accelH = std::experimental::nullopt,
    .bar    = std::experimental::nullopt,
    .ggr    = std::experimental::nullopt,
    .gps    = std::experimental::nullopt,
    .gyro   = std::experimental::make_optional(gyroReading),
    .mag    = std::experimental::nullopt
  };

  // Update the world estimate
  WorldEstimate world = worldEstimator.update(meas);

  // Poll for controller input
  ControllerInput input = inputSource.read();

  // Run the controllers
  ActuatorSetpoint actuatorSp;
  if(isArmed()) {
    // Run the controller pipeline as determined by the subclass
    AngularVelocitySetpoint sp {
      .rollVel = input.roll,
      .pitchVel = input.pitch,
      .yawVel = input.yaw,
      .throttle = input.throttle
    };
    actuatorSp = pipeline.run(world, sp, attVelController, attAccController);
  } else {
    // Run the zero controller
    actuatorSp = zeroController.run(world, actuatorSp);
  }

  // Update motor outputs
  motorMapper.run(isArmed(), actuatorSp);
}

void CarVehicleSystem::on(const protocol::message::set_arm_state_message_t& m) {
  setArmed(m.armed);
}
