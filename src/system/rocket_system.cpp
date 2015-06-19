#include "system/rocket_system.hpp"
#include "util/time.hpp"

#include "ch.hpp"
#include "chprintf.h"

RocketSystem::RocketSystem(
    Accelerometer& accel,
    optional<Accelerometer *> accelH,
    optional<Barometer *> bar,
    optional<Geiger *> ggr,
    optional<GPS *> gps,
    Gyroscope& gyr,
    optional<Magnetometer *> mag,
    WorldEstimator& estimator, InputSource& inputSource,
    MotorMapper& motorMapper, Communicator& communicator, Logger& logger,
    Platform& platform)
  : VehicleSystem(communicator), MessageListener(communicator),
    accel(accel), accelH(accelH), bar(bar), ggr(ggr), gps(gps), gyr(gyr), mag(mag),
    estimator(estimator), inputSource(inputSource),
    motorMapper(motorMapper), platform(platform),
    systemStream(communicator, 10),
    logger(logger),
    state(RocketState::DISARMED) {
  // Disarm by default. A set_arm_state_message_t message is required to enable
  // the control pipeline.
  setArmed(false);
}

void RocketSystem::update() {
  //static int time = 0;
  //if (time % 1000 == 0) {
  //  chprintf((BaseSequentialStream*)&SD4, "%d\r\n", RTT2MS(chibios_rt::System::getTime()));
  //}
  //time = (time+1) % 1000;

  // Poll sensors
  AccelerometerReading accelReading = accel.readAccel();
  GyroscopeReading gyrReading = gyr.readGyro();
  optional<AccelerometerReading> accelHReading;
  optional<BarometerReading> barReading;
  optional<GeigerReading> ggrReading;
  optional<GPSReading> gpsReading;
  optional<MagnetometerReading> magReading;

  if (accelH) accelHReading = (*accelH)->readAccel();
  if (bar)    barReading    = (*bar)->readBar();
  if (ggr)    ggrReading    = (*ggr)->readGeiger();
  if (gps)    gpsReading    = (*gps)->readGPS();
  if (mag)    magReading    = (*mag)->readMag();

  SensorMeasurements meas {
    .accel  = std::experimental::make_optional(accelReading),
    .accelH = accelHReading,
    .bar    = barReading,
    .ggr    = ggrReading,
    .gps    = gpsReading,
    .gyro   = std::experimental::make_optional(gyrReading),
    .mag    = magReading
  };

  // Update the world estimate
  WorldEstimate estimate = estimator.update(meas);

  // Run the controllers
  ActuatorSetpoint actuatorSp = {0,0,0,0};

  // Run state machine
  switch (state) {
  case RocketState::DISARMED:
    state = DisarmedState(meas, estimate, actuatorSp);
    break;
  case RocketState::PRE_ARM:
    state = PreArmState(meas, estimate, actuatorSp);
    break;
  case RocketState::ARMED:
    state = ArmedState(meas, estimate, actuatorSp);
    break;
  case RocketState::FLIGHT:
    state = FlightState(meas, estimate, actuatorSp);
    break;
  case RocketState::APOGEE:
    state = ApogeeState(meas, estimate, actuatorSp);
    break;
  case RocketState::DESCENT:
    state = DescentState(meas, estimate, actuatorSp);
    break;
  case RocketState::RECOVERY:
    state = RecoveryState(meas, estimate, actuatorSp);
    break;
  default:
    break;
  }

  // Update motor outputs
  motorMapper.run(isArmed(), actuatorSp);

  // Poll for controller input
  ControllerInput input = inputSource.read();

  // Update streams
  updateStreams(meas, estimate, actuatorSp);
}

bool RocketSystem::healthy() {
  bool healthy = accel.healthy() && gyr.healthy();

  if(accelH) {
    healthy &= (*accelH)->healthy();
  }

  if(bar) {
    healthy &= (*bar)->healthy();
  }

  if(ggr) {
    healthy &= (*ggr)->healthy();
  }

  if(gps) {
    healthy &= (*gps)->healthy();
  }

  if(mag) {
    healthy &= (*mag)->healthy();
  }

  return healthy;
}

void RocketSystem::on(const protocol::message::set_arm_state_message_t& m) {
  if (state == RocketState::DISARMED) {
    return;
  }
  setArmed(m.armed);
}

void RocketSystem::updateStreams(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  protocol::message::system_message_t m {
    .time = ST2MS(chibios_rt::System::getTime()),
    .state = (int) state,
    .motorDC = sp.throttle
  };
  logger.write(m);
  if (systemStream.ready()) {
    systemStream.publish(m);
  }
}

RocketState RocketSystem::DisarmedState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  PulseLED(0,1,0,1);   // Green 1 Hz

  static bool calibrated = false;
  static int calibCount = 0;
  static std::array<float, 3> gyrOffsets {unit_config::GYR_X_OFFSET, unit_config::GYR_Y_OFFSET, unit_config::GYR_Z_OFFSET};
  static std::array<float, 3> accOffsets {unit_config::ACC_X_OFFSET, unit_config::ACC_Y_OFFSET, unit_config::ACC_Z_OFFSET};
  static std::array<float, 3> acchOffsets {unit_config::ACCH_X_OFFSET, unit_config::ACCH_Y_OFFSET, unit_config::ACCH_Z_OFFSET};

  // Calibrate ground altitude
  groundAltitude = est.loc.alt;

  // Calibrate accelerometers
  // TODO(yoos): Implement calibration routine once we have persistent params
  accel.setOffsets(accOffsets);
  (*accelH)->setOffsets(acchOffsets);

  // Calibrate gyroscope
  for (int i=0; i<3; i++) {
    gyrOffsets[i] = (gyrOffsets[i]*calibCount + (*meas.gyro).axes[i])/(calibCount+1);
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

    calibrated = true;
  }

  // Proceed to PRE_ARM if calibration done
  if (calibrated) {
    return RocketState::PRE_ARM;
  }

  return RocketState::DISARMED;
}

RocketState RocketSystem::PreArmState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  PulseLED(0,1,0,4);   // Green 4 Hz

  // Proceed to ARMED if all sensors are healthy and GS arm signal received.
  if (healthy() && isArmed()) {
    return RocketState::ARMED;
  }

  return RocketState::PRE_ARM;
}

RocketState RocketSystem::ArmedState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  SetLED(0,1,0);   // Green

  // Revert to PRE_ARM if any sensors are unhealthy or disarm signal received
  if (!(healthy() && isArmed())) {
    return RocketState::PRE_ARM;
  }

  // Proceed to FLIGHT on 1.2g sense on X axis.
  static int count = 10;
  count = ((*meas.accel).axes[0] > 1.2) ? (count-1) : 10;
  if (count == 0) {
    return RocketState::FLIGHT;
  }

  return RocketState::ARMED;
}

RocketState RocketSystem::FlightState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  SetLED(0,0,1);   // Blue
  static bool powered = true;   // First time we enter, we are in powered flight.

  // Check for motor cutoff. We should see negative acceleration due to drag.
  static int count = 100;
  if (powered) {
    count = ((*meas.accel).axes[0] < 0.0) ? (count-1) : 100;
    if (count == 0) powered = false;
  }

  // Apogee occurs after motor cutoff and at near-zero altitude change towards
  // end of ascent (ideal case)
  if (!powered && est.loc.dAlt < 2.0) {
    // If falling faster than -40m/s for whatever reason, definitely deploy.
    if (est.loc.dAlt < -40.0) {
      return RocketState::APOGEE;
    }
    // Otherwise, also check that we are not just undergoing a subsonic
    // transition.
    else if ((*meas.accel).axes[0] > -1.0) {
      return RocketState::APOGEE;
    }
  }

  return RocketState::FLIGHT;
}

RocketState RocketSystem::ApogeeState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  PulseLED(0,0,1,2);   // Blue 2 Hz
  static float sTime = 0.0;   // State time

  // Fire drogue pyro
  platform.get<DigitalPlatform>().set(PIN_DROGUE_CH, true);

  // Count continuous time under drogue
  // TODO(yoos): We might still see this if partially deployed and spinning
  // around..
  static float drogueTime = 0.0;
  if ((*meas.accel).axes[0] > 0.3) {
    drogueTime += unit_config::DT;
  }
  else {
    drogueTime = 0.0;
  }

  // Check for successful drogue deployment. If failure detected, deploy main.
  if (sTime < 10.0) {   // TODO(yoos): Is it safe to wait this long?
    if (drogueTime > 1.0) {   // TODO(yoos): Do we want to wait longer to ensure drogue?
      return RocketState::DESCENT;
    }
  }
  else {
    platform.get<DigitalPlatform>().set(PIN_MAIN_CH, true);
    return RocketState::DESCENT;
  }

  sTime += unit_config::DT;
  return RocketState::APOGEE;
}

RocketState RocketSystem::DescentState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  PulseLED(1,0,1,1);   // Violet 1 Hz
  static float sTime = 0.0;   // State time

  // Deploy main at 1500' (457.2m) AGL.
  if (est.loc.alt < (groundAltitude + 457.2)) {
    platform.get<DigitalPlatform>().set(PIN_MAIN_CH, true);
  }

  // Stay for at least 1 s
  static int count = 1000;
  if (sTime < 1.0) {}
  // Enter recovery if altitude is unchanging and rotation rate is zero
  else if (est.loc.dAlt > -2.0 &&
      fabs((*meas.gyro).axes[0] < 0.05) &&
      fabs((*meas.gyro).axes[1] < 0.05) &&
      fabs((*meas.gyro).axes[2] < 0.05)) {
    count -= 1;
  }
  else {
    count = 1000;
  }

  if (count == 0) {
    return RocketState::RECOVERY;
  }

  sTime += unit_config::DT;
  return RocketState::DESCENT;
}

RocketState RocketSystem::RecoveryState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  PulseLED(1,1,1,2);   // White 2 Hz

  // Turn things off
  platform.get<DigitalPlatform>().set(PIN_MAIN_CH, false);
  platform.get<DigitalPlatform>().set(PIN_DROGUE_CH, false);

  return RocketState::RECOVERY;
}

void RocketSystem::SetLED(float r, float g, float b) {
  platform.get<PWMPlatform>().set(9,  0.1*r);
  platform.get<PWMPlatform>().set(10, 0.1*g);
  platform.get<PWMPlatform>().set(11, 0.1*b);
}

void RocketSystem::BlinkLED(float r, float g, float b, float freq) { static int count = 0;
  int period = 1000 / freq;
  if (count % period < period/2) {
    SetLED(r,g,b);
  }
  else {
    SetLED(0,0,0);
  }
  count = (count+1) % period;
}

void RocketSystem::PulseLED(float r, float g, float b, float freq) {
  int period = 1000 / freq;
  static int count = 0;
  float dc = ((float) abs(period/2 - count)) / (period/2);
  SetLED(dc*r, dc*g, dc*b);
  count = (count+1) % period;
}

