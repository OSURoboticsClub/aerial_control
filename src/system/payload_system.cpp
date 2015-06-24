#include "system/payload_system.hpp"
#include "util/time.hpp"

#include "ch.hpp"
#include "chprintf.h"

PayloadSystem::PayloadSystem(
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
    state(PayloadState::DISARMED) {
  // Disarm by default. A set_arm_state_message_t message is required to enable
  // the control pipeline.
  setArmed(false);
  gyr.setAxisConfig(unit_config::GYR_AXES);
  accel.setAxisConfig(unit_config::ACC_AXES);
  (*accelH)->setAxisConfig(unit_config::ACCH_AXES);
  gyr.setOffsets(unit_config::GYR_OFFSETS);
  accel.setOffsets(unit_config::ACC_OFFSETS);
  (*accelH)->setOffsets(unit_config::ACCH_OFFSETS);
}

void PayloadSystem::update() {
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
  case PayloadState::DISARMED:
    state = DisarmedState(meas, estimate, actuatorSp);
    break;
  case PayloadState::PRE_ARM:
    state = PreArmState(meas, estimate, actuatorSp);
    break;
  case PayloadState::ARMED:
    state = ArmedState(meas, estimate, actuatorSp);
    break;
  case PayloadState::FLIGHT:
    state = FlightState(meas, estimate, actuatorSp);
    break;
  case PayloadState::APOGEE:
    state = ApogeeState(meas, estimate, actuatorSp);
    break;
  case PayloadState::MICROGRAVITY:
    state = MicrogravityState(meas, estimate, actuatorSp);
    break;
  case PayloadState::DESCENT:
    state = DescentState(meas, estimate, actuatorSp);
    break;
  case PayloadState::RECOVERY:
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

bool PayloadSystem::healthy() {
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

void PayloadSystem::on(const protocol::message::set_arm_state_message_t& m) {
  if (state == PayloadState::DISARMED) {
    return;
  }
  setArmed(m.armed);
}

void PayloadSystem::updateStreams(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  protocol::message::system_message_t m {
    .time = ST2MS(chibios_rt::System::getTime()),
    .state = (uint8_t) state,
    .motorDC = sp.throttle
  };
  logger.write(m);
  if (systemStream.ready()) {
    systemStream.publish(m);
  }
}

PayloadState PayloadSystem::DisarmedState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  PulseLED(0,1,0,4);   // Green 4 Hz

  static bool calibrated = false;
  static int calibCount = 0;
  static std::array<float, 3> gyrOffsets = unit_config::GYR_OFFSETS;

  // Calibrate ground altitude
  groundAltitude = est.loc.alt;

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

    calibrated = true;
  }

  // Proceed to PRE_ARM if calibration done
  if (calibrated) {
    return PayloadState::PRE_ARM;
  }

  return PayloadState::DISARMED;
}

PayloadState PayloadSystem::PreArmState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  PulseLED(0,1,0,1);   // Green 1 Hz

  // Proceed to ARMED if all sensors are healthy, arm signal is received, and
  // rocket is within 15 degrees of vertical.
  if (healthy() && isArmed() && (est.att.pitch > M_PI/12)) {
    return PayloadState::ARMED;
  }

  return PayloadState::PRE_ARM;
}

PayloadState PayloadSystem::ArmedState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  SetLED(0,1,0);   // Green

  // Revert to PRE_ARM if any arm conditions are violated
  if (!(healthy() && isArmed() && (est.att.pitch > M_PI/12))) {
    return PayloadState::PRE_ARM;
  }

  // Proceed to FLIGHT on 1.2g sense on X axis.
  static int count = 10;
  count = ((*meas.accel).axes[0] > 1.2) ? (count-1) : 10;
  if (count == 0) {
    return PayloadState::FLIGHT;
  }

  return PayloadState::ARMED;
}

PayloadState PayloadSystem::FlightState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  SetLED(0,0,1);   // Blue
  static bool powered = true;   // First time we enter, we are in powered flight.

  // Check for motor cutoff. We should see negative acceleration due to drag.
  static int count = 100;
  if (powered) {
    count = ((*meas.accel).axes[0] < 0.0) ? (count-1) : 100;
    if (count == 0) powered = false;
  }

  // Apogee occurs after motor cutoff. Payload apogee detection is simplified
  // to detecting the sudden forward acceleration caused by the main separation
  // event triggered by the rocket avionics.
  if (!powered) {
    static uint8_t mainSepCount = 10;
    mainSepCount = ((*meas.accel).axes[0] > 0.0) ? mainSepCount-1 : 10;
    if (mainSepCount == 0) {
      return PayloadState::APOGEE;
    }
  }

  return PayloadState::FLIGHT;
}

PayloadState PayloadSystem::ApogeeState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  PulseLED(0,0,1,2);   // Blue 2 Hz
  static float sTime = 0.0;   // State time

  // Wait to clear rocket
  if (sTime < 2.0) {
    // Noop
  }
  else {
    return PayloadState::MICROGRAVITY;
  }

  sTime += unit_config::DT;
  return PayloadState::APOGEE;
}

PayloadState PayloadSystem::MicrogravityState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  RGBLED(2);   // Rainbows!
  static float sTime = 0.0;   // State time
  static float throttle = 0.0;

  // Run microgravity maneuver for 6 s.
  if (sTime < 4.0) {
    if ((*meas.accel).axes[0] < 0.0 && throttle < 1.0) {
      throttle += 0.001;
    }
    else if ((*meas.accel).axes[0] > 0.0 && throttle > 0.0) {
      throttle -= 0.001;
    }
    if (throttle < 0.0) throttle = 0.0;
    if (throttle > 1.0) throttle = 1.0;
  }
  else if (sTime < 5.0) {
    throttle = 0.0;
  }
  else if (sTime < 10.0) {
    // Fire drogue pyro
    platform.get<DigitalPlatform>().set(PIN_MAIN_CH, true);
    if ((*meas.accel).axes[0] < -5.0) {   // Large negative acceleration due to proper drogue deployment
      return PayloadState::DESCENT;
    }
  }
  else {
    // If payload had one, we would deploy the main chute here.
    return PayloadState::DESCENT;
  }

  sTime += unit_config::DT;

  // Outputs
  sp.throttle = throttle;
  return PayloadState::MICROGRAVITY;
}

PayloadState PayloadSystem::DescentState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  PulseLED(1,0,1,1);   // Violet 1 Hz
  static float sTime = 0.0;   // State time

  // If payload had one, we would deploy main at 1500' AGL.

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
    return PayloadState::RECOVERY;
  }

  sTime += unit_config::DT;
  return PayloadState::DESCENT;
}

PayloadState PayloadSystem::RecoveryState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  PulseLED(1,1,1,2);   // White 2 Hz

  // Turn things off
  platform.get<DigitalPlatform>().set(PIN_MAIN_CH, false);

  return PayloadState::RECOVERY;
}

void PayloadSystem::SetLED(float r, float g, float b) {
  platform.get<PWMPlatform>().set(9,  0.1*r);
  platform.get<PWMPlatform>().set(10, 0.1*g);
  platform.get<PWMPlatform>().set(11, 0.1*b);
}

void PayloadSystem::BlinkLED(float r, float g, float b, float freq) {
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

void PayloadSystem::PulseLED(float r, float g, float b, float freq) {
  int period = 1000 / freq;
  static int count = 0;
  float dc = ((float) abs(period/2 - count)) / (period/2);
  SetLED(dc*r, dc*g, dc*b);
  count = (count+1) % period;
}

void PayloadSystem::RGBLED(float freq) {
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
