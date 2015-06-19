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
    Communicator& communicator,
    Logger& logger,
    Platform& platform)
  : VehicleSystem(communicator),
    MessageListener(communicator),
    gyr(gyr), acc(acc), bar(bar), gps(gps), mag(mag),
    estimator(estimator),
    inputSource(inputSource),
    motorMapper(motorMapper),
    platform(platform),
    logger(logger),
    mode(MultirotorControlMode::ANGULAR_POS) {
  // Disarm by default. A set_arm_state_message_t message is required to enable
  // the control pipeline.
  setArmed(false);
  gyr.setAxisConfig(unit_config::GYR_AXES);
  acc.setAxisConfig(unit_config::ACC_AXES);
  acc.setOffsets(unit_config::ACC_OFFSETS);
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
        SetLED(0,1,1);
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
        SetLED(0,1,0);
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
        SetLED(1,0,1);
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
    PulseLED(0,1,0,1);
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
void MultirotorVehicleSystem::SetLED(float r, float g, float b) {
  platform.get<PWMPlatform>().set(9,  r);
  platform.get<PWMPlatform>().set(10, g);
  platform.get<PWMPlatform>().set(11, b);
}

void MultirotorVehicleSystem::BlinkLED(float r, float g, float b, float freq) { static int count = 0;
  int period = 1000 / freq;
  if (count % period < period/2) {
    SetLED(r,g,b);
  }
  else {
    SetLED(0,0,0);
  }
  count = (count+1) % period;
}

void MultirotorVehicleSystem::PulseLED(float r, float g, float b, float freq) {
  int period = 1000 / freq;
  static int count = 0;
  float dc = ((float) abs(period/2 - count)) / (period/2);
  SetLED(dc*r, dc*g, dc*b);
  count = (count+1) % period;
}

void MultirotorVehicleSystem::RGBLED(float freq) {
  static float dc = 0.0;
  static int dir = 1;
  if (dc >= 1.0) {
    dir = -1;
  }
  else if (dc <= 0.0) {
    dir = 1;
  }
  dc += dir * (2*freq * unit_config::DT);

  float dc_ = dc;
  float dir_ = dir;
  for (int i=0; i<3; i++) {
    dc_ += dir_ * 0.666;
    if (dc_ > 1.0) {
      dc_ = 2.0 - dc_;
      dir_ = -1;
    }
    else if (dc_ < 0.0) {
      dc_ = 0.0 - dc_;
      dir_ = 1;
    }
    platform.get<PWMPlatform>().set(9+i, dc_);
  }
}
