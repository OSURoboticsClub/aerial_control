#include "system/multirotor_vehicle_system.hpp"

#include "unit_config.hpp"
#include "input/ppm_input_source.hpp"
#include "util/math.hpp"

MultirotorVehicleSystem::MultirotorVehicleSystem(
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
    posController(params),
    attPosController(params),
    attVelController(params),
    attAccController(params),
    motorMapper(motorMapper),
    platform(platform),
    stream(communicator, 10), logger(logger),
    mode(MultirotorControlMode::CALIBRATION) {
  // Disarm by default. A set_arm_state_message_t message is required to enable
  // the control pipeline.
  setArmed(false);
}

void MultirotorVehicleSystem::update() {
  // Poll sensors
  SensorMeasurements meas = sensors.readAvailableSensors();

  // Update world estimate
  WorldEstimate estimate = estimator.update(meas);

  // Poll for controller input
  ControllerInput input = inputSource.read();

  // Ensure sensors are calibrated and healthy before arming, but don't disarm
  // if sensors become unhealthy.
  // TODO(syoo): Checking health at 1kHz is expensive. Fix.
  if (input.valid) {
    if (healthy() && sensors.calibrated()) {
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
  } else if(!sensors.calibrated()) {
    mode = MultirotorControlMode::CALIBRATION;
  } else {
    mode = MultirotorControlMode::DISARMED;
  }

  // Run the controllers
  ActuatorSetpoint actuatorSp = {0, 0, 0, 0};
  switch (mode) {
    case MultirotorControlMode::CALIBRATION:
      CalibrationMode();
      break;
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

bool MultirotorVehicleSystem::healthy() const {
  return sensors.healthy();
}

void MultirotorVehicleSystem::on(const protocol::message::set_arm_state_message_t& m) {
  setArmed(m.armed);
}

void MultirotorVehicleSystem::CalibrationMode() {
  PulseLED(0,1,0,4);   // FIXME: This interferes with disarmed mode LED stuff..

  sensors.calibrateStep();
  if (sensors.calibrated()) {
    mode = MultirotorControlMode::DISARMED;
  }
}

void MultirotorVehicleSystem::DisarmedMode(SensorMeasurements meas, WorldEstimate est, ControllerInput input, ActuatorSetpoint& sp) {
  PulseLED(0,1,0,1);
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
  yawPosSp += M_PI * input.yaw * params.get(GlobalParameters::PARAM_DT);
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
  yawPosSp += input.yaw * params.get(GlobalParameters::PARAM_DT);
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
  dc += dir * (2*freq * params.get(GlobalParameters::PARAM_DT));

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
