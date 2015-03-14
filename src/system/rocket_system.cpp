#include "system/rocket_system.hpp"

#include "chprintf.h"

RocketSystem::RocketSystem(
    Accelerometer& accel,
    optional<Accelerometer *> accelH,
    optional<Barometer *> bar,
    optional<GPS *> gps,
    Gyroscope& gyr,
    optional<Magnetometer *> mag,
    WorldEstimator& estimator, InputSource& inputSource,
    MotorMapper& motorMapper, Communicator& communicator,
    PWMPlatform& pwmPlatform)
  : VehicleSystem(communicator), MessageListener(communicator),
    accel(accel), accelH(accelH), bar(bar), gps(gps), gyr(gyr), mag(mag),
    estimator(estimator), inputSource(inputSource),
    motorMapper(motorMapper), pwmPlatform(pwmPlatform) {
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
  ActuatorSetpoint actuatorSp;

  // Run state machine
  static RocketState state = RocketState::DISARMED;
  switch (state) {
  case RocketState::DISARMED:
    state = DisarmedState(meas, estimate);
    break;
  case RocketState::PRE_ARM:
    state = PreArmState(meas, estimate);
    break;
  case RocketState::ARMED:
    state = ArmedState(meas, estimate);
    break;
  case RocketState::FLIGHT:
    state = FlightState(meas, estimate);
    break;
  case RocketState::APOGEE:
    state = ApogeeState(meas, estimate);
    break;
  case RocketState::DESCENT:
    state = DescentState(meas, estimate);
    break;
  case RocketState::RECOVERY:
    state = RecoveryState(meas, estimate);
    break;
  default:
    break;
  }

  // Update motor outputs
  motorMapper.run(isArmed(), actuatorSp);
}

bool RocketSystem::healthy() {
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

void RocketSystem::on(const protocol::message::set_arm_state_message_t& m) {
  setArmed(m.armed);
}

RocketState RocketSystem::DisarmedState(SensorMeasurements meas, WorldEstimate est) {
  PulseLED(1,0,0,1);   // Green 1 Hz

  // Proceed directly to PRE_ARM for now.
  return RocketState::PRE_ARM;
}

RocketState RocketSystem::PreArmState(SensorMeasurements meas, WorldEstimate est) {
  PulseLED(1,0,0,4);   // Green 4 Hz

  // Proceed to ARMED if all sensors are healthy and GS arm signal received.
  if (healthy() && isArmed()) {
    return RocketState::ARMED;
  }

  return RocketState::PRE_ARM;
}

RocketState RocketSystem::ArmedState(SensorMeasurements meas, WorldEstimate est) {
  SetLED(1,0,0);   // Green

  static int count = 10;
  count = ((*meas.accel).axes[0] > 1.1) ? (count-1) : 10;

  // Revert to PRE_ARM if any sensors are unhealthy or disarm signal received
  if (!(healthy() && isArmed())) {
    return RocketState::PRE_ARM;
  }

  // Proceed to FLIGHT on 1.1g sense on X axis.
  if (count == 0) {
    return RocketState::FLIGHT;
  }

  return RocketState::ARMED;
}

RocketState RocketSystem::FlightState(SensorMeasurements meas, WorldEstimate est) {
  SetLED(0,0,1);   // Blue
  static bool powered = true;   // First time we enter, we are in powered flight.

  // Check for motor cutoff.
  if (powered && (*meas.accel).axes[0] < 0.5) {
    powered = false;
  }

  // Apogee occurs after motor cutoff
  if (!powered) {
    // If falling faster than -40m/s, definitely deploy.
    if (est.loc.dAlt < -40.0) {
      return RocketState::APOGEE;
    }
    // Check for zero altitude change. This is the ideal case.
    else if (est.loc.dAlt < 0.0) {
      // Check we are not just undergoing a subsonic transition
      if (!((*meas.accel).axes[0] > -1.0)) {
        return RocketState::APOGEE;
      }
    }
  }

  return RocketState::FLIGHT;
}

RocketState RocketSystem::ApogeeState(SensorMeasurements meas, WorldEstimate est) {
  PulseLED(0,0,1,1);   // Blue 1 Hz

  return RocketState::APOGEE;
}

RocketState RocketSystem::DescentState(SensorMeasurements meas, WorldEstimate est) {
  SetLED(1,0,1);   // Yellow

  return RocketState::DESCENT;
}

RocketState RocketSystem::RecoveryState(SensorMeasurements meas, WorldEstimate est) {
  PulseLED(1,0,1,2);   // White 2 Hz

  return RocketState::RECOVERY;
}

void RocketSystem::SetLED(float r, float g, float b) {
  pwmPlatform.set(9, r);
  pwmPlatform.set(10, g);
  pwmPlatform.set(11, b);
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

void RocketSystem::RGBLED(float freq) {
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
      pwmPlatform.set(5+i, dc_*0.05);
    }
  }
}
