#include "system/multirotor_vehicle_system.hpp"

MultirotorVehicleSystem::MultirotorVehicleSystem(
    Gyroscope& gyroscope,
    Accelerometer& accelerometer,
    GPS& gps,
    optional<Magnetometer *> magnetometer,
    optional<Thermistor *> thermistor,
    WorldEstimator& world,
    AttitudeEstimator& attitude,
    InputSource& inputSource,
    MotorMapper& motorMapper,
    Communicator& communicator)
  : VehicleSystem(communicator),
    MessageListener(communicator),
    gyroscope(gyroscope),
    accelerometer(accelerometer),
    gps(gps),
    magnetometer(magnetometer),
    thermistor(thermistor),
    world(world),
    attitude(attitude),
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

  // Only use magnetometer if it is available
  if(magnetometer) {
    magReading = (*magnetometer)->readMag();
  }

  gps_reading_t gpsReading;
  optional<ThermistorReading> thermReading;
  static int i=0;
  if (i++ % 100 == 0) {
    gpsReading = gps.readGPS();
    if (thermistor) {
      thermReading = (*therm)->readTherm();
    }
  }

  // TODO: Currently copying all readings
  SensorReadingGroup readings {
    .gyro = std::experimental::make_optional(gyroReading),
    .accel = std::experimental::make_optional(accelReading),
    .mag = magReading,
    .gps = gpsReading,
    .therm = thermReading
  };

  // Update estimates
  AttitudeEstimate attitude_estimate = attitude.update(readings);
  WorldEstimate world_estimate = world.update(readings);

  // Poll for controller input
  ControllerInput input = inputSource.read();

  // Run the controllers
  ActuatorSetpoint actuatorSp;
  if(isArmed() && input.valid) {
    // Run the controller pipeline as determined by the subclass
    switch(mode) {
      case MultirotorControlMode::POSITION: {
        PositionSetpoint sp {
          .latitude = input.roll,
          .longitude = input.pitch,
          .yawPos = input.yaw,
          .altitude = input.throttle
        };
        actuatorSp = pipeline.run(attitude_estimate, sp, posController, attPosController, attVelController, attAccController);
        break;
      }
      case MultirotorControlMode::VELOCITY: {
        // TODO: implement
        // actuatorSp = pipeline.run(attitude_estimate, sp, velController, attPosController, attVelController, attAccController);
        break;
      }
      case MultirotorControlMode::ANGULAR_POS: {
        AngularPositionSetpoint sp {
          .rollPos = input.roll,
          .pitchPos = input.pitch,
          .yawPos = input.yaw,
          .throttle = input.throttle
        };
        actuatorSp = pipeline.run(attitude_estimate, sp, attPosController, attVelController, attAccController);
        break;
      }
      case MultirotorControlMode::ANGULAR_RATE: {
        AngularVelocitySetpoint sp {
          .rollVel = input.roll,
          .pitchVel = input.pitch,
          .yawVel = input.yaw,
          .throttle = input.throttle
        };
        actuatorSp = pipeline.run(attitude_estimate, sp, attVelController, attAccController);
        break;
      }
    }
  } else {
    // Run the zero controller
    actuatorSp = zeroController.run(attitude_estimate, actuatorSp);
  }

  // Update motor outputs
  motorMapper.run(isArmed(), actuatorSp);
}

void MultirotorVehicleSystem::on(const protocol::message::set_arm_state_message_t& m) {
  setArmed(m.armed);
}
