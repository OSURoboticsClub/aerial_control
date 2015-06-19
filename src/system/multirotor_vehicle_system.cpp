#include "system/multirotor_vehicle_system.hpp"

MultirotorVehicleSystem::MultirotorVehicleSystem(
    Gyroscope& gyr,
    Accelerometer& acc,
    optional<Barometer *> bar,
    optional<GPS *> gps,
    optional<Magnetometer *> mag,
    WorldEstimator& estimator,
    InputSource& inputSource,
    MotorMapper& motorMapper,
    Communicator& communicator)
  : VehicleSystem(communicator),
    MessageListener(communicator),
    gyr(gyr), acc(acc), bar(bar), gps(gps), mag(mag),
    estimator(estimator),
    inputSource(inputSource),
    motorMapper(motorMapper),
    mode(MultirotorControlMode::ANGULAR_POS) {
  // Disarm by default. A set_arm_state_message_t message is required to enable
  // the control pipeline.
  setArmed(false);
  gyr.setAxisConfig(unit_config::GYR_AXES);
  acc.setAxisConfig(unit_config::ACC_AXES);
}

void MultirotorVehicleSystem::update() {
  // Poll the gyroscope and accelerometer
  GyroscopeReading gyroReading = gyr.readGyro();
  AccelerometerReading accelReading = acc.readAccel();
  optional<BarometerReading> barReading;
  optional<GPSReading> gpsReading;
  optional<MagnetometerReading> magReading;

  if (bar) barReading = (*bar)->readBar();
  if (gps) gpsReading = (*gps)->readGPS();
  if (mag) magReading = (*mag)->readMag();

  // TODO: Currently copying all readings
  SensorMeasurements meas {
    .accel  = std::experimental::make_optional(accelReading),
    .accelH = std::experimental::nullopt,
    .bar    = std::experimental::nullopt,
    .ggr    = std::experimental::nullopt,
    .gps    = gpsReading,
    .gyro   = std::experimental::make_optional(gyroReading),
    .mag    = magReading
  };

  // Update estimates
  WorldEstimate world = estimator.update(meas);

  // Poll for controller input
  ControllerInput input = inputSource.read();

  // Run the controllers
  ActuatorSetpoint actuatorSp;
  if (isArmed() && input.valid) {
    // Run the controller pipeline as determined by the subclass
    switch(mode) {
      case MultirotorControlMode::POSITION: {
        PositionSetpoint sp {
          .lat = input.roll,
          .lon = input.pitch,
          .yawPos = input.yaw,
          .alt = input.throttle
        };
        actuatorSp = pipeline.run(world, sp, posController, attPosController, attVelController, attAccController);
        break;
      }
      case MultirotorControlMode::VELOCITY: {
        // TODO: implement
        // actuatorSp = pipeline.run(world, sp, velController, attPosController, attVelController, attAccController);
        break;
      }
      case MultirotorControlMode::ANGULAR_POS: {
        AngularPositionSetpoint sp {
          .rollPos = input.roll,
          .pitchPos = input.pitch,
          .yawPos = input.yaw,
          .throttle = input.throttle
        };
        actuatorSp = pipeline.run(world, sp, attPosController, attVelController, attAccController);
        break;
      }
      case MultirotorControlMode::ANGULAR_RATE: {
        AngularVelocitySetpoint sp {
          .rollVel = input.roll,
          .pitchVel = input.pitch,
          .yawVel = input.yaw,
          .throttle = input.throttle
        };
        actuatorSp = pipeline.run(world, sp, attVelController, attAccController);
        break;
      }
    }
  } else {
    // Run the zero controller
    actuatorSp = zeroController.run(world, actuatorSp);
  }

  // Update motor outputs
  motorMapper.run(isArmed(), actuatorSp);
}

bool MultirotorVehicleSystem::healthy() {
  bool healthy = gyr.healthy() && acc.healthy();

  if(bar) {
    healthy &= (*bar)->healthy();
  }

  if(gps) {
    healthy &= (*gps)->healthy();
  }

  if(mag) {
    healthy &= (*mag)->healthy();
  }

  return healthy;
}

void MultirotorVehicleSystem::on(const protocol::message::set_arm_state_message_t& m) {
  setArmed(m.armed);
}
