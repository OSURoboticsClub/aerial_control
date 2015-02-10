#include "controller/angular_acceleration_controller.hpp"

#include <algorithm>

#include "unit_config.hpp"

AngularAccelerationController::AngularAccelerationController()
  : rollAccPid(unit_config::ANGACC_X_KP, unit_config::ANGACC_X_KI, unit_config::ANGACC_X_KD),
    pitchAccPid(unit_config::ANGACC_Y_KP, unit_config::ANGACC_Y_KI, unit_config::ANGACC_Y_KD),
    yawAccPid(unit_config::ANGACC_Z_KP, unit_config::ANGACC_Z_KI, unit_config::ANGACC_Z_KD) {
}

ActuatorSetpoint AngularAccelerationController::run(const AttitudeEstimate& estimate, const AngularAccelerationSetpoint& input) {
  // Limit to maximum angular accelerations
  float rollAccSp = std::max(-unit_config::MAX_PITCH_ROLL_ACC, std::min(unit_config::MAX_PITCH_ROLL_ACC, input.rollAcc));
  float pitchAccSp = std::max(-unit_config::MAX_PITCH_ROLL_ACC, std::min(unit_config::MAX_PITCH_ROLL_ACC, input.pitchAcc));

  // Run PID controllers
  float rollActuatorSp = rollAccPid.calculate(rollAccSp, estimate.rollAcc, unit_config::DT);
  float pitchActuatorSp = pitchAccPid.calculate(pitchAccSp, estimate.pitchAcc, unit_config::DT);
  float yawActuatorSp = yawAccPid.calculate(input.yawAcc, estimate.yawAcc, unit_config::DT);

  // Output
  ActuatorSetpoint setpoint {
    .roll = rollActuatorSp,
    .pitch = pitchActuatorSp,
    .yaw = yawActuatorSp,
    .throttle = input.throttle
  };

  return setpoint;
}
