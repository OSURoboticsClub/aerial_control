#include "system/payload_system.hpp"
#include "util/time.hpp"

#include "ch.hpp"
#include "chprintf.h"

PayloadSystem::PayloadSystem(
    Accelerometer& accel,
    optional<Accelerometer *> accelH,
    optional<Barometer *> bar,
    optional<GPS *> gps,
    Gyroscope& gyr,
    optional<Magnetometer *> mag,
    WorldEstimator& estimator, InputSource& inputSource,
    MotorMapper& motorMapper, Communicator& communicator,
    Platform& platform)
  : VehicleSystem(communicator), MessageListener(communicator),
    accel(accel), accelH(accelH), bar(bar), gps(gps), gyr(gyr), mag(mag),
    estimator(estimator), inputSource(inputSource),
    motorMapper(motorMapper), platform(platform),
    imuStream(communicator, 10),   // TODO(yoos): calculate data link budget and increase if possible
    systemStream(communicator, 5),
    state(PayloadState::DISARMED) {
  // Disarm by default. A set_arm_state_message_t message is required to enable
  // the control pipeline.
  setArmed(false);
}

void PayloadSystem::update() {
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
  optional<GPSReading> gpsReading;
  optional<MagnetometerReading> magReading;

  if (accelH) accelHReading = (*accelH)->readAccel();
  if (bar)    barReading    = (*bar)->readBar();
  if (gps)    gpsReading    = (*gps)->readGPS();
  //if (mag)    magReading    = (*mag)->readMag();

  SensorMeasurements meas {
    .accel  = std::experimental::make_optional(accelReading),
    .accelH = accelHReading,
    .bar    = barReading,
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
  case PayloadState::ZERO_G:
    state = ZeroGState(meas, estimate, actuatorSp);
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
    //healthy &= (*accelH)->healthy();
  }

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

void PayloadSystem::on(const protocol::message::set_arm_state_message_t& m) {
  setArmed(m.armed);
}

void PayloadSystem::updateStreams(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  if (imuStream.ready()) {
    protocol::message::imu_message_t m {
      .time = ST2MS(chibios_rt::System::getTime()),
      .gyro = {
        (*meas.gyro).axes[0],
        (*meas.gyro).axes[1],
        (*meas.gyro).axes[2]
      },
      .accel = {
        (*meas.accel).axes[0],
        (*meas.accel).axes[1],
        (*meas.accel).axes[2]
      }
    };

    imuStream.publish(m);
  }

  if (systemStream.ready()) {
    uint8_t stateNum = 0;
    switch (state) {
    case PayloadState::DISARMED:
      stateNum = 0;
      break;
    case PayloadState::PRE_ARM:
      stateNum = 1;
      break;
    case PayloadState::ARMED:
      stateNum = 2;
      break;
    case PayloadState::FLIGHT:
      stateNum = 3;
      break;
    case PayloadState::APOGEE:
      stateNum = 4;
      break;
    case PayloadState::ZERO_G:
      stateNum = 5;
      break;
    case PayloadState::DESCENT:
      stateNum = 6;
      break;
    case PayloadState::RECOVERY:
      stateNum = 7;
      break;
    default:
      break;
    }

    protocol::message::system_message_t m {
      .time = ST2MS(chibios_rt::System::getTime()),
      .state = stateNum,
      .motorDC = sp.throttle
    };

    systemStream.publish(m);
  }
}

PayloadState PayloadSystem::DisarmedState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  PulseLED(1,0,0,1);   // Red 1 Hz

  // Proceed directly to PRE_ARM for now.
  return PayloadState::PRE_ARM;
}

PayloadState PayloadSystem::PreArmState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  PulseLED(1,0,0,4);   // Red 4 Hz

  // Calibrate
  groundAltitude = est.loc.alt;
  // TODO(yoos): run sensor calibration here

  // Proceed to ARMED if all sensors are healthy and GS arm signal received.
  if (healthy() && isArmed()) {
    return PayloadState::ARMED;
  }

  return PayloadState::PRE_ARM;
}

PayloadState PayloadSystem::ArmedState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  SetLED(1,0,0);   // Red

  static int count = 10;
  count = ((*meas.accel).axes[0] > 1.1) ? (count-1) : 10;

  // Revert to PRE_ARM if any sensors are unhealthy or disarm signal received
  if (!(healthy() && isArmed())) {
    return PayloadState::PRE_ARM;
  }

  // Proceed to FLIGHT on 1.1g sense on X axis.
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
    powered = (count == 0) ? false : true;
  }

  // Apogee occurs after motor cutoff
  if (!powered) {
    // If falling faster than -40m/s, definitely deploy.
    if (est.loc.dAlt < -40.0) {
      return PayloadState::APOGEE;
    }
    // Check for near-zero altitude change towards end of ascent (ideal case)
    // and that we are not just undergoing a subsonic transition.
    else if ((*meas.accel).axes[0] > -1.0) {
      return PayloadState::APOGEE;
    }
  }

  return PayloadState::FLIGHT;
}

PayloadState PayloadSystem::ApogeeState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  PulseLED(0,0,1,2);   // Blue 2 Hz
  static float sTime = 0.0;   // State time

  // Count continuous time under drogue
  // TODO(yoos): We might still see this if partially deployed and spinning
  // around..
  static float drogueTime = 0.0;

  // Wait until successful drogue deployment or 5 seconds max.
  if (sTime < 5.0) {
    if ((*meas.accel).axes[0] < -0.3) {
      drogueTime += unit_config::DT;
    }
    else {
      drogueTime = 0.0;
    }

    if (drogueTime > 1.0) {   // TODO(yoos): Do we want to wait longer for ensure drogue?
      return PayloadState::ZERO_G;
    }
    // TODO: What if payload doesn't separate?
  }
  else {
    return PayloadState::ZERO_G;
  }

  sTime += unit_config::DT;
  return PayloadState::APOGEE;
}

PayloadState PayloadSystem::ZeroGState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  RGBLED(2);   // Rainbows!
  static float sTime = 0.0;   // State time
  static float throttle = 0.0;

  // Release shuttle
  platform.get<DigitalPlatform>().set(PIN_SHUTTLE1_CH, true);
  platform.get<DigitalPlatform>().set(PIN_SHUTTLE2_CH, true);

  // Run zero-g maneuver for 6 s.
  if (sTime < 0.5) {
    // Noop until we clear the shuttle and rocket
  }
  else if (sTime < 6.0) {
    if ((*meas.accel).axes[0] < 0.0 && throttle < 1.0) {
      throttle += 0.001;
    }
    else if ((*meas.accel).axes[0] > 0.0 && throttle > 0.0) {
      throttle -= 0.001;
    }
    if (throttle < 0.0) throttle = 0.0;
    if (throttle > 1.0) throttle = 1.0;
  }
  else if (sTime < 7.0) {
    throttle = 0.0;
  }
  else if (sTime < 10.0) {
    // Fire drogue pyro
    platform.get<DigitalPlatform>().set(PIN_DROGUE_CH, true);
    if ((*meas.accel).axes[0] < -10.0) {   // Large negative acceleration due to proper drogue deployment
      return PayloadState::DESCENT;
    }
  }
  else {
    // We would deploy main here, but payload avionics doesn't have room.
    return PayloadState::DESCENT;
  }

  sTime += unit_config::DT;

  // Outputs
  sp.throttle = throttle;
  return PayloadState::ZERO_G;
}

PayloadState PayloadSystem::DescentState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  SetLED(1,0,1);   // Violet
  static float sTime = 0.0;   // State time

  // Payload does not control its own main.

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
  PulseLED(1,0,1,2);   // Violet 2 Hz

  // Turn things off
  platform.get<DigitalPlatform>().set(PIN_DROGUE_CH, false);

  return PayloadState::RECOVERY;
}

void PayloadSystem::SetLED(float r, float g, float b) {
  platform.get<PWMPlatform>().set(9,  0.1*r);
  platform.get<PWMPlatform>().set(10, 0.1*g);
  platform.get<PWMPlatform>().set(11, 0.1*b);
}

void PayloadSystem::BlinkLED(float r, float g, float b, float freq) { static int count = 0;
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
