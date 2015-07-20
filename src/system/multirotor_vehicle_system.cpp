#include "system/multirotor_vehicle_system.hpp"

MultirotorVehicleSystem::MultirotorVehicleSystem(
    Gyroscope& gyroscope,
    Accelerometer& accelerometer,
    optional<GPS *> gps,
    optional<Magnetometer *> magnetometer,
    WorldEstimator& estimator,
    InputSource& inputSource,
    MotorMapper& motorMapper,
    Communicator& communicator)
  : VehicleSystem(communicator),
    MessageListener(communicator),
    gyroscope(gyroscope),
    accelerometer(accelerometer),
    gps(gps),
    magnetometer(magnetometer),
    estimator(estimator),
    inputSource(inputSource),
    motorMapper(motorMapper),
    mode(MultirotorControlMode::ANGULAR_POS) {
  // Disarm by default. A set_arm_state_message_t message is required to enable
  // the control pipeline.
  setArmed(false);
}

void MultirotorVehicleSystem::update() {
  // Poll the gyroscope and accelerometer
  GyroscopeReading gyroReading = gyroscope.readGyro();
  AccelerometerReading accelReading = accelerometer.readAccel();
  optional<MagnetometerReading> magReading;
  optional<GPSReading> gpsReading;

  // Only use magnetometer if it is available
  if(magnetometer) {
    magReading = (*magnetometer)->readMag();
  }

  if (gps) {
    static int i=0;
    if (i++ % 100 == 0) {
      // TODO(yoos): GPS read makes board freeze
      //gpsReading = (*gps)->readGPS();
    }
  }

  // TODO: Currently copying all readings
  SensorMeasurements meas {
    .accel  = std::experimental::make_optional(accelReading),
    .accelH = std::experimental::nullopt,
    .bar    = std::experimental::nullopt,
    .gps    = gpsReading,
    .gyro   = std::experimental::make_optional(gyroReading),
    .mag    = magReading
  };

  // Update estimates
  WorldEstimate world = estimator.update(meas);

  // Poll for controller input
  ControllerInput input = inputSource.read();

  if(input.valid && input.armed) {
    // Ensure sensors are healthy before arming, but don't disarm if sensors
    // become unhealthy.
    if(healthy()) {
      setArmed(true);
    }
  } else {
    setArmed(false);
  }

  // Run the controllers
  ActuatorSetpoint actuatorSp;
  if(isArmed() && input.valid) {
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
  bool healthy = gyroscope.healthy() && accelerometer.healthy();

  if(gps) {
    healthy &= (*gps)->healthy();
  }

  if(magnetometer) {
    healthy &= (*magnetometer)->healthy();
  }

  return healthy;
}

void MultirotorVehicleSystem::on(const protocol::message::set_arm_state_message_t& m) {
  setArmed(m.armed);
}
