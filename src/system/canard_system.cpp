#include "unit_config.hpp"
#include "system/canard_system.hpp"
#include "util/time.hpp"

#include "ch.hpp"
#include "chprintf.h"

CanardSystem::CanardSystem(
    ParameterRepository& params,
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
    params(params),
    accel(accel), accelH(accelH), bar(bar), ggr(ggr), gps(gps), gyr(gyr), mag(mag),
    estimator(estimator), inputSource(inputSource),
    attPosController(params),
    attVelController(params),
    attAccController(params),
    motorMapper(motorMapper), platform(platform),
    systemStream(communicator, 10),
    logger(logger),
    state(CanardState::DISARMED) {
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

void CanardSystem::update() {
  //static int time = 0;
  //if (time % 1000 == 0) {
  //  chprintf((BaseSequentialStream*)&SD4, "%d\r\n", RTT2MS(chibios_rt::System::getTime()));
  //}
  //time = (time+1) % 1000;

  // Poll the gyroscope and accelerometer
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
  //if (mag)    magReading    = (*mag)->readMag();

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
  case CanardState::DISARMED:
    state = DisarmedState(meas, estimate, actuatorSp);
    break;
  case CanardState::PRE_ARM:
    state = PreArmState(meas, estimate, actuatorSp);
    break;
  case CanardState::ARMED:
    state = ArmedState(meas, estimate, actuatorSp);
    break;
  case CanardState::FLIGHT:
    state = FlightState(meas, estimate, actuatorSp);
    break;
  case CanardState::APOGEE:
    state = ApogeeState(meas, estimate, actuatorSp);
    break;
  case CanardState::DESCENT:
    state = DescentState(meas, estimate, actuatorSp);
    break;
  case CanardState::RECOVERY:
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

bool CanardSystem::healthy() {
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

void CanardSystem::on(const protocol::message::set_arm_state_message_t& m) {
  setArmed(m.armed);
}

void CanardSystem::updateStreams(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
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

CanardState CanardSystem::DisarmedState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
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

  static bool finCheckComplete = false;
  static float finCheckDutyCycle = 0.001;
  static float finCheckSpeed = 0.001;
  static int finCheckStep = 0;

  // Wiggle fins
  if (!finCheckComplete) {
    switch (finCheckStep) {
    case 0:
      finCheckDutyCycle += finCheckSpeed;
      if (finCheckDutyCycle >= 1) {
        finCheckStep = 1;
      }
      break;
    case 1:
      finCheckDutyCycle -= finCheckSpeed;
      if (finCheckDutyCycle <= 0) {
        finCheckStep = 2;
      }
      break;
    case 2:
      finCheckDutyCycle += finCheckSpeed;
      if (finCheckDutyCycle >= 0.5) {
        finCheckComplete = true;
      }
    default:
      break;
    }
  }
  sp.roll = finCheckDutyCycle;

  // Proceed to PRE_ARM if calibration done
  if (calibrated && finCheckComplete) {
    return CanardState::PRE_ARM;
  }

  return CanardState::DISARMED;
}

CanardState CanardSystem::PreArmState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  PulseLED(0,1,0,1);   // Green 1 Hz

  static int buzzcount = 0;
  platform.get<PWMPlatform>().set(PIN_BUZZER, 0.05 * (buzzcount<50));
  buzzcount = (buzzcount+1) % 2000;

  // Proceed to ARMED if all sensors are healthy and GS arm signal received.
  if (healthy() && isArmed()) {
    return CanardState::ARMED;
  }

  sp.roll = 0.3;
  return CanardState::PRE_ARM;
}

CanardState CanardSystem::ArmedState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  SetLED(0,1,0);   // Green

  // Turn off buzzer
  platform.get<PWMPlatform>().set(PIN_BUZZER, 0);

  static int count = 10;
  count = ((*meas.accel).axes[0] > 1.2) ? (count-1) : 10;

  // Revert to PRE_ARM if any sensors are unhealthy or disarm signal received
  if (!(healthy() && isArmed())) {
    return CanardState::PRE_ARM;
  }

  // Proceed to FLIGHT on 1.2g sense on X axis.
  if (count == 0) {
    return CanardState::FLIGHT;
  }

  sp.roll = 0.5;
  return CanardState::ARMED;
}

CanardState CanardSystem::FlightState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  SetLED(0,0,1);   // Blue
  static bool powered = true;   // First time we enter, we are in powered flight.

  // Check for motor cutoff. We should see negative acceleration due to drag.
  if (powered) {
    static int count = 100;
    count = ((*meas.accel).axes[0] < -0.05) ? (count-1) : 100;
    powered = (count == 0) ? false : true;
  }

  // Apogee occurs after motor cutoff
  if (!powered &&
      est.loc.dAlt < 2.0 &&
      (*meas.accel).axes[0] > -0.05) {
    return CanardState::APOGEE;
  }

  // Run controller
  static float flightTime = 0.0;
  if (flightTime < 3.0) {
    AngularVelocitySetpoint velSp { 3.14159, 0, 0, 0 };
    sp = pipeline.run(est, velSp, attVelController, attAccController);
  }
  else if (flightTime < 6.0) {
    AngularVelocitySetpoint velSp { -3.14159, 0, 0, 0 };
    sp = pipeline.run(est, velSp, attVelController, attAccController);
  }
  else if (flightTime < 10.0) {
    AngularVelocitySetpoint velSp { 0, 0, 0, 0 };
    sp = pipeline.run(est, velSp, attVelController, attAccController);
  }
  else {
    AngularVelocitySetpoint velSp { 0, 0, 0, 0 };
    sp = pipeline.run(est, velSp, attVelController, attAccController);
  }
  flightTime += params.get(GlobalParameters::PARAM_DT);

  return CanardState::FLIGHT;
}

CanardState CanardSystem::ApogeeState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  PulseLED(0,0,1,2);   // Blue 2 Hz
  static float sTime = 0.0;   // State time

  // Count continuous time under drogue
  // TODO(yoos): We might still see this if partially deployed and spinning
  // around..
  static float drogueTime = 0.0;
  if ((*meas.accel).axes[0] < -0.3) {
    drogueTime += params.get(GlobalParameters::PARAM_DT);
  }
  else {
    drogueTime = 0.0;
  }

  // Check for successful drogue deployment. For the canard system, which does
  // not fire pyros, this is just to see if we can.
  if (sTime < 10.0) {
    if (drogueTime > 1.0) {
      return CanardState::DESCENT;
    }
  }
  else {
    return CanardState::DESCENT;
  }

  sTime += params.get(GlobalParameters::PARAM_DT);
  sp.roll = 0.5;
  return CanardState::APOGEE;
}

CanardState CanardSystem::DescentState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  PulseLED(1,0,1,1);   // Violet 1 Hz
  static float sTime = 0.0;   // State time

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
    return CanardState::RECOVERY;
  }

  sTime += params.get(GlobalParameters::PARAM_DT);
  sp.roll = 0.5;
  return CanardState::DESCENT;
}

CanardState CanardSystem::RecoveryState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  PulseLED(1,1,1,2);   // White 2 Hz

  // Beep loudly
  static int buzzcount = 0;
  if (buzzcount < 50 || (buzzcount > 100 && buzzcount < 150)) {
    platform.get<PWMPlatform>().set(PIN_BUZZER, 0.2);
  }
  else {
    platform.get<PWMPlatform>().set(PIN_BUZZER, 0);
  }
  buzzcount = (buzzcount+1) % 2000;

  sp.roll = 0.5;
  return CanardState::RECOVERY;
}

void CanardSystem::SetLED(float r, float g, float b) {
  platform.get<PWMPlatform>().set(9,  0.1*r);
  platform.get<PWMPlatform>().set(10, 0.1*g);
  platform.get<PWMPlatform>().set(11, 0.1*b);
}

void CanardSystem::BlinkLED(float r, float g, float b, float freq) {
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

void CanardSystem::PulseLED(float r, float g, float b, float freq) {
  int period = 1000 / freq;
  static int count = 0;
  float dc = ((float) abs(period/2 - count)) / (period/2);
  SetLED(dc*r, dc*g, dc*b);
  count = (count+1) % period;
}

