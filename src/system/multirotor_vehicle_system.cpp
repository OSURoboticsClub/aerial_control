#include "system/multirotor_vehicle_system.hpp"

#include <input/ppm_input_source.hpp>
#include <util/math.hpp>

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
    stream(communicator, 10), logger(logger),
    mode(MultirotorControlMode::DISARMED),
    calibrated(false),
    altSp(0.0),
    yawPosSp(0.0) {
  // Disarm by default. A set_arm_state_message_t message is required to enable
  // the control pipeline.
  setArmed(false);
  gyr.setAxisConfig(unit_config::GYR_AXES);
  acc.setAxisConfig(unit_config::ACC_AXES);
  gyr.setOffsets(unit_config::GYR_OFFSETS);
  acc.setOffsets(unit_config::ACC_OFFSETS);
}

void MultirotorVehicleSystem::update() {
  // Poll sensors
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
    .bar    = barReading,
    .ggr    = std::experimental::nullopt,
    .gps    = gpsReading,
    .gyro   = std::experimental::make_optional(gyroReading),
    .mag    = magReading
  };

  // Update world estimate
  WorldEstimate estimate = estimator.update(meas);

  // Poll for controller input
  ControllerInput input = inputSource.read();

  // Ensure sensors are calibrated and healthy before arming, but don't disarm
  // if sensors become unhealthy.
  // TODO(syoo): Checking health at 1kHz is expensive. Fix.
  if (input.valid) {
    if (healthy() && calibrated) {
      setArmed(input.armed);
    }
    else if (input.armed == false) {
      setArmed(false);
    }
  }
  else {
    setArmed(false);
  }

  // Set mode
  if (isArmed()) {
    // AUTO
    //if (input.mode == (int)InputControlMode::AUTO) {
    if (false) {
      mode = MultirotorControlMode::ANGULAR_POS;   // TODO(yoos): AUTO disabled until we implement controller
    }
    // MANUAL or ALTCTL
    else {
      if (input.velocityMode) {
        mode = MultirotorControlMode::ANGULAR_VEL;
      }
      else {
        mode = MultirotorControlMode::ANGULAR_POS;
      }
    }
  }
  else {
    mode = MultirotorControlMode::DISARMED;
  }

  // Run the controllers
  ActuatorSetpoint actuatorSp = {0, 0, 0, 0};
  switch (mode) {
    case MultirotorControlMode::DISARMED:
      DisarmedMode(meas, estimate, input, actuatorSp);
      break;
    case MultirotorControlMode::ANGULAR_VEL:
      AngularRateMode(meas, estimate, input, actuatorSp);
      break;
    case MultirotorControlMode::ANGULAR_POS:
      AngularPosMode(meas, estimate, input, actuatorSp);
      break;
    case MultirotorControlMode::POSITION:
      PosMode(meas, estimate, input, actuatorSp);
      break;
    default:
      break;
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

void MultirotorVehicleSystem::calibrate(SensorMeasurements meas) {
  PulseLED(0,1,0,2);   // FIXME: This interferes with disarmed mode LED stuff..
  static int calibCount = 0;
  static std::array<float, 3> gyrOffsets = unit_config::GYR_OFFSETS;

  // Calibrate ground altitude
  //groundAltitude = est.loc.alt;

  // Calibrate gyroscope
  for (int i=0; i<3; i++) {
    gyrOffsets[i] = (gyrOffsets[i]*calibCount + (*meas.gyro).axes[i]+unit_config::GYR_OFFSETS[i])/(calibCount+1);
  }
  calibCount++;

  // Reset calibration on excessive gyration
  if (fabs((*meas.gyro).axes[0] > 0.1) ||
      fabs((*meas.gyro).axes[1] > 0.1) ||
      fabs((*meas.gyro).axes[2] > 0.1)) {
    calibCount = 0;
  }

  // Run calibration for 5 seconds
  if (calibCount == 5000) {
    gyr.setOffsets(gyrOffsets);
    protocol::message::sensor_calibration_response_message_t m_gyrcal {
      .type = protocol::message::sensor_calibration_response_message_t::SensorType::GYRO,
      .offsets = {gyrOffsets[0], gyrOffsets[1], gyrOffsets[2]}
    };
    logger.write(m_gyrcal);
    if (stream.ready()) {
      stream.publish(m_gyrcal);
    }

    calibrated = true;
  }
}

void MultirotorVehicleSystem::DisarmedMode(SensorMeasurements meas, WorldEstimate est, ControllerInput input, ActuatorSetpoint& sp) {
  PulseLED(0,1,0,1);
  if (!calibrated) {
    calibrate(meas);
  }

  // Run the zero controller
  ActuatorSetpoint zSp = {0, 0, 0, 0};
  sp = zeroController.run(est, zSp);
}

void MultirotorVehicleSystem::AngularRateMode(SensorMeasurements meas, WorldEstimate est, ControllerInput input, ActuatorSetpoint& sp) {
  PulseLED(0,0,1,8);
  altSp = est.loc.alt;   // Reset altitude setpoint
  yawPosSp = est.att.yaw;   // Reset yaw position setpoint to current yaw position while in velocity mode

  AngularVelocitySetpoint avSp {
    .rollVel  = M_PI*input.roll,
    .pitchVel = M_PI*input.pitch,
    .yawVel   = M_PI*input.yaw,
    .throttle = input.throttle
  };
  sp = pipeline.run(est, avSp, attVelController, attAccController);
}

void MultirotorVehicleSystem::AngularPosMode(SensorMeasurements meas, WorldEstimate est, ControllerInput input, ActuatorSetpoint& sp) {
  SetLED(0,0,1);
  yawPosSp += M_PI * input.yaw * unit_config::DT;
  if (yawPosSp > M_PI)  {yawPosSp -= 2*M_PI;}
  if (yawPosSp < -M_PI) {yawPosSp += 2*M_PI;}

  AngularPositionSetpoint apSp {
    .rollPos  = input.roll,
    .pitchPos = input.pitch,
    .yawPos   = yawPosSp,
    .throttle = input.throttle
  };

  // Reset altitude setpoint only if not in ALTCTL mode
  if (input.mode != (int)InputControlMode::ALTCTL) {
    altSp = est.loc.alt;
  }
  // Somewhat hacky way to achieve altitude hold. Pilot should first find
  // a good hover throttle, then switch to ALTCTL.
  else {
    float throttleShift = std::min(std::max(altSp - est.loc.alt, -0.2f), 0.2f);
    apSp.throttle += throttleShift;
  }

  // TODO(yoos): Adjust roll based on yaw. Servo angle is coupled with
  // roll. That is, we need the output of the acceleration controller to
  // change the input to the position controller. How can we do this?
  sp = pipeline.run(est, apSp, attPosController, attVelController, attAccController);
}

void MultirotorVehicleSystem::PosMode(SensorMeasurements meas, WorldEstimate est, ControllerInput input, ActuatorSetpoint& sp) {
  SetLED(0,1,1);
  yawPosSp += input.yaw * unit_config::DT;
  if (yawPosSp > M_PI)  {yawPosSp -= 2*M_PI;}
  if (yawPosSp < -M_PI) {yawPosSp += 2*M_PI;}

  PositionSetpoint pSp {
    .lat    = est.loc.lat,
    .lon    = est.loc.lon,
    .yawPos = yawPosSp,
    .alt    = est.loc.alt
  };
  sp = pipeline.run(est, pSp, posController, attPosController, attVelController, attAccController);
}

void MultirotorVehicleSystem::SetLED(float r, float g, float b) {
  platform.get<PWMPlatform>().set(9,  r);
  platform.get<PWMPlatform>().set(10, g);
  platform.get<PWMPlatform>().set(11, b);
}

void MultirotorVehicleSystem::BlinkLED(float r, float g, float b, float freq) {
  static int count = 0;
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
