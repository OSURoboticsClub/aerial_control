#include "unit_config.hpp"
#include "system/canard_system.hpp"
#include "util/time.hpp"

#include "ch.hpp"
#include "chprintf.h"

CanardSystem::CanardSystem(
    ParameterRepository& params,
    Sensors& sensors,
    WorldEstimator& estimator,
    InputSource& inputSource,
    MotorMapper& motorMapper,
    Communicator& communicator,
    Logger& logger,
    Platform& platform)
  : VehicleSystem(communicator),
    MessageListener(communicator),
    params(params),
    sensors(sensors),
    estimator(estimator),
    inputSource(inputSource),
    attPosController(params),
    attVelController(params),
    attAccController(params),
    motorMapper(motorMapper),
    platform(platform),
    systemStream(communicator, 10),
    logger(logger),
    state(CanardState::DISARMED) {
  // Disarm by default. A set_arm_state_message_t message is required to enable
  // the control pipeline.
  setArmed(true);
}

void CanardSystem::update() {
  //static int time = 0;
  //if (time % 1000 == 0) {
  //  chprintf((BaseSequentialStream*)&SD4, "%d\r\n", RTT2MS(chibios_rt::System::getTime()));
  //}
  //time = (time+1) % 1000;

  // Poll the gyroscope and accelerometer
  SensorMeasurements meas = sensors.readAvailableSensors();

  // Update the world estimate
  WorldEstimate estimate = estimator.update(meas);

  // Run the controllers
  ActuatorSetpoint actuatorSp = {0,0,0.5,0};

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

  // DEBUG
  //AngularPositionSetpoint posSp {0, 0, 0, 0};
  //actuatorSp = pipeline.run(estimate, posSp, attPosController, attVelController, attAccController);

  // Update motor outputs
  motorMapper.run(isArmed(), actuatorSp);

  // Poll for controller input
  ControllerInput input = inputSource.read();

  // Update streams
  updateStreams(meas, estimate, actuatorSp);
}

bool CanardSystem::healthy() {
  return sensors.healthy();
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

  // Calibrate sensors
  sensors.calibrateStep();
  if (sensors.calibrated()) {
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
  sp.yaw = finCheckDutyCycle;

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

  sp.yaw = 0.3;
  return CanardState::PRE_ARM;
}

CanardState CanardSystem::ArmedState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  SetLED(0,1,0);   // Green

  // Turn off buzzer
  platform.get<PWMPlatform>().set(PIN_BUZZER, 0);

  static int count = 10;
  count = ((*meas.accel).axes[2] > 2.0) ? (count-1) : 10;

  // Revert to PRE_ARM if any sensors are unhealthy or disarm signal received
  if (!(healthy() && isArmed())) {
    return CanardState::PRE_ARM;
  }

  // Proceed to FLIGHT on 2.0g sense on Z axis.
  if (count == 0) {
    return CanardState::FLIGHT;
  }

  sp.yaw = 0.5;
  return CanardState::ARMED;
}

CanardState CanardSystem::FlightState(SensorMeasurements meas, WorldEstimate est, ActuatorSetpoint& sp) {
  SetLED(0,0,1);   // Blue
  static bool powered = true;   // First time we enter, we are in powered flight.

  // Check for motor cutoff. We should see negative acceleration due to drag.
  if (powered) {
    static int count = 100;
    count = ((*meas.accel).axes[2] < -0.05) ? (count-1) : 100;
    powered = (count == 0) ? false : true;
  }

  // Apogee occurs after motor cutoff
  if (!powered &&
      est.loc.dAlt < 2.0 &&
      (*meas.accel).axes[2] > -0.05) {
    return CanardState::APOGEE;
  }

  // Run controller
  static float flightTime = 0.0;
  if (flightTime < 3.0) {
    sp.yaw = 0.0;
  }
  else if (flightTime < 5.0) {
    sp.yaw = 1.0;
  }
  else if (flightTime < 7.0) {
    sp.yaw = 0.5;
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
  if ((*meas.accel).axes[2] < -0.3) {
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
  sp.yaw = 0.5;
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
  sp.yaw = 0.5;
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

  sp.yaw = 0.5;
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

