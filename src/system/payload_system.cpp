#include "system/payload_system.hpp"

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
    imuStream(communicator, 100) {
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

  // Poll for controller input
  ControllerInput input = inputSource.read();

  // Keep moving average of acceleration
  static float accel = -1.0f;
  accel = 0.5*accel + 0.5*accelReading.axes[0];

  // Run the controllers
  //ActuatorSetpoint actuatorSp;

  // Run state machine
  static PayloadState state = PayloadState::DISARMED;
  switch (state) {
  case PayloadState::DISARMED:
    state = DisarmedState(meas, estimate);
    break;
  case PayloadState::PRE_ARM:
    state = PreArmState(meas, estimate);
    break;
  case PayloadState::ARMED:
    state = ArmedState(meas, estimate);
    break;
  case PayloadState::FLIGHT:
    state = FlightState(meas, estimate);
    break;
  case PayloadState::APOGEE:
    state = ApogeeState(meas, estimate);
    break;
  case PayloadState::DESCENT:
    state = DescentState(meas, estimate);
    break;
  case PayloadState::RECOVERY:
    state = RecoveryState(meas, estimate);
    break;
  default:
    break;
  }

  // Update streams
  updateStreams(meas, estimate);

  // Update motor outputs
  //motorMapper.run(isArmed(), actuatorSp);
}

bool PayloadSystem::healthy() {
  bool healthy = accel.healthy() && gyr.healthy();

  if(accelH) {
    healthy &= (*accelH)->healthy();
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

void PayloadSystem::updateStreams(SensorMeasurements meas, WorldEstimate est) {
  if (imuStream.ready()) {
    protocol::message::imu_message_t m {
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
}

PayloadState PayloadSystem::DisarmedState(SensorMeasurements meas, WorldEstimate est) {
  PulseLED(1,0,0,1);   // Red 1 Hz

  // Proceed directly to PRE_ARM for now.
  return PayloadState::PRE_ARM;
}

PayloadState PayloadSystem::PreArmState(SensorMeasurements meas, WorldEstimate est) {
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

PayloadState PayloadSystem::ArmedState(SensorMeasurements meas, WorldEstimate est) {
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

PayloadState PayloadSystem::FlightState(SensorMeasurements meas, WorldEstimate est) {
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
    // Check for near-zero altitude change towards end of ascent. This is the ideal case.
    else if (est.loc.dAlt < 2.0) {
      // Check we are not just undergoing a subsonic transition
      if ((*meas.accel).axes[0] > -1.0) {
        return PayloadState::APOGEE;
      }
    }
  }

  return PayloadState::FLIGHT;
}

PayloadState PayloadSystem::ApogeeState(SensorMeasurements meas, WorldEstimate est) {
  PulseLED(0,0,1,2);   // Blue 2 Hz
  static float sTime = 0.0;   // State time

  // Fire drogue pyro
  platform.get<DigitalPlatform>().set(unit_config::PIN_DROGUE_CH, true);

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
    if (drogueTime > 1.0) {   // TODO(yoos): Do we want to wait longer for ensure drogue?
      return PayloadState::DESCENT;
    }
  }
  else {
    platform.get<DigitalPlatform>().set(unit_config::PIN_MAIN_CH, true);
    return PayloadState::DESCENT;
  }

  sTime += unit_config::DT;
  return PayloadState::APOGEE;
}

PayloadState PayloadSystem::DescentState(SensorMeasurements meas, WorldEstimate est) {
  SetLED(1,0,1);   // Violet
  static float sTime = 0.0;   // State time

  // Deploy main at 1500' (457.2m) AGL.
  if (est.loc.alt < (groundAltitude + 457.2)) {
    platform.get<DigitalPlatform>().set(unit_config::PIN_MAIN_CH, true);
  }

  // Stay for at least 1 s
  if (sTime < 1.0) {}
  // Enter recovery if altitude is unchanging and rotation rate is zero
  else if (est.loc.dAlt > -2.0 &&
      fabs((*meas.gyro).axes[0] < 0.1) &&
      fabs((*meas.gyro).axes[1] < 0.1) &&
      fabs((*meas.gyro).axes[2] < 0.1)) {
    return PayloadState::RECOVERY;
  }

  sTime += unit_config::DT;
  return PayloadState::DESCENT;
}

PayloadState PayloadSystem::RecoveryState(SensorMeasurements meas, WorldEstimate est) {
  PulseLED(1,0,1,2);   // Violet 2 Hz

  // Turn things off
    platform.get<DigitalPlatform>().set(unit_config::PIN_MAIN_CH, false);
    platform.get<DigitalPlatform>().set(unit_config::PIN_DROGUE_CH, false);

  return PayloadState::RECOVERY;
}

void PayloadSystem::SetLED(float r, float g, float b) {
  platform.get<PWMPlatform>().set(9, r);
  platform.get<PWMPlatform>().set(10, g);
  platform.get<PWMPlatform>().set(11, b);
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
  float dc = 0.0;
  int dir = 1;
  while(true) {
    if (dc >= 1.0) {
      dir = -1;
    }
    else if (dc <= 0.0) {
      dir = 1;
    }
    dc += dir * 0.02;

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
      platform.get<PWMPlatform>().set(5+i, dc_*0.05);
    }
  }
}
