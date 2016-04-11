#include "controller/angular_position_controller.hpp"

#include <algorithm>

#include "global_parameters.hpp"
#include "util/math.hpp"

AngularPositionController::AngularPositionController(ParameterRepository& params)
  : params(params) {
  params.def(PARAM_PID_ROLL_KP, 0.0);
  params.def(PARAM_PID_ROLL_KI, 0.0);
  params.def(PARAM_PID_ROLL_KD, 0.0);
  params.def(PARAM_PID_PITCH_KP, 0.0);
  params.def(PARAM_PID_PITCH_KI, 0.0);
  params.def(PARAM_PID_PITCH_KD, 0.0);
  params.def(PARAM_PID_YAW_KP, 0.0);
  params.def(PARAM_PID_YAW_KI, 0.0);
  params.def(PARAM_PID_YAW_KD, 0.0);
  params.def(PARAM_MAX_PITCH_ROLL_POS, 0.0);
}

AngularVelocitySetpoint AngularPositionController::run(const WorldEstimate& world, const AngularPositionSetpoint& input) {
  // Limit to maximum angles
  float rollPosSp = std::max(-params.get(PARAM_MAX_PITCH_ROLL_POS), std::min(params.get(PARAM_MAX_PITCH_ROLL_POS), input.rollPos));
  float pitchPosSp = std::max(-params.get(PARAM_MAX_PITCH_ROLL_POS), std::min(params.get(PARAM_MAX_PITCH_ROLL_POS), input.pitchPos));
  float yawPosSp = input.yawPos;

  // Favor the short rotation (less than 180 deg) to target
  if      (rollPosSp  - world.att.roll  >  M_PI) rollPosSp  -= 2*M_PI;
  else if (rollPosSp  - world.att.roll  < -M_PI) rollPosSp  += 2*M_PI;
  if      (pitchPosSp - world.att.pitch >  M_PI) pitchPosSp -= 2*M_PI;
  else if (pitchPosSp - world.att.pitch < -M_PI) pitchPosSp += 2*M_PI;
  if      (yawPosSp   - world.att.yaw   >  M_PI) yawPosSp   -= 2*M_PI;
  else if (yawPosSp   - world.att.yaw   < -M_PI) yawPosSp   += 2*M_PI;

  // Run PID controllers
  float rollVelSp = rollPosPid.calculate(rollPosSp, world.att.roll, params.get(GlobalParameters::PARAM_DT));
  float pitchVelSp = pitchPosPid.calculate(pitchPosSp, world.att.pitch, params.get(GlobalParameters::PARAM_DT));
  float yawVelSp = yawPosPid.calculate(yawPosSp, world.att.yaw, params.get(GlobalParameters::PARAM_DT));

  // Output
  AngularVelocitySetpoint setpoint {
    .rollVel = rollVelSp,
    .pitchVel = pitchVelSp,
    .yawVel = yawVelSp,
    .throttle = input.throttle
  };

  return setpoint;
}
