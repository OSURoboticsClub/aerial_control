#include "controller/angular_position_controller.hpp"

#include <algorithm>

#include <util/math.hpp>
#include "unit_config.hpp"

AngularPositionController::AngularPositionController()
  : rollPosPid(unit_config::ANGPOS_X_KP, unit_config::ANGPOS_X_KI, unit_config::ANGPOS_X_KD),
    pitchPosPid(unit_config::ANGPOS_Y_KP, unit_config::ANGPOS_Y_KI, unit_config::ANGPOS_Y_KD),
    yawPosPid(unit_config::ANGPOS_Z_KP, unit_config::ANGPOS_Z_KI, unit_config::ANGPOS_Z_KD) {
}

AngularVelocitySetpoint AngularPositionController::run(const WorldEstimate& world, const AngularPositionSetpoint& input) {
  // Limit to maximum angles
  float rollPosSp = std::max(-unit_config::MAX_PITCH_ROLL_POS, std::min(unit_config::MAX_PITCH_ROLL_POS, input.rollPos));
  float pitchPosSp = std::max(-unit_config::MAX_PITCH_ROLL_POS, std::min(unit_config::MAX_PITCH_ROLL_POS, input.pitchPos));
  float yawPosSp = input.yawPos;

  // Favor the short rotation (less than 180 deg) to target
  if      (rollPosSp  - world.att.roll  >  M_PI) rollPosSp  -= 2*M_PI;
  else if (rollPosSp  - world.att.roll  < -M_PI) rollPosSp  += 2*M_PI;
  if      (pitchPosSp - world.att.pitch >  M_PI) pitchPosSp -= 2*M_PI;
  else if (pitchPosSp - world.att.pitch < -M_PI) pitchPosSp += 2*M_PI;
  if      (yawPosSp   - world.att.yaw   >  M_PI) yawPosSp   -= 2*M_PI;
  else if (yawPosSp   - world.att.yaw   < -M_PI) yawPosSp   += 2*M_PI;

  // Run PID controllers
  float rollVelSp = rollPosPid.calculate(rollPosSp, world.att.roll, unit_config::DT);
  float pitchVelSp = pitchPosPid.calculate(pitchPosSp, world.att.pitch, unit_config::DT);
  float yawVelSp = yawPosPid.calculate(yawPosSp, world.att.yaw, unit_config::DT);

  // Output
  AngularVelocitySetpoint setpoint {
    .rollVel = rollVelSp,
    .pitchVel = pitchVelSp,
    .yawVel = yawVelSp,
    .throttle = input.throttle
  };

  return setpoint;
}
