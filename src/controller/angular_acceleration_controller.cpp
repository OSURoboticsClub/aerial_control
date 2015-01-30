#include "controller/angular_acceleration_controller.hpp"

#include <algorithm>

#include "unit_config.hpp"

AngularAccelerationController::AngularAccelerationController()
  : rollAccPid(unit_config::ANGACC_X_KP, unit_config::ANGACC_X_KI, unit_config::ANGACC_X_KD),
    pitchAccPid(unit_config::ANGACC_Y_KP, unit_config::ANGACC_Y_KI, unit_config::ANGACC_Y_KD),
    yawAccPid(unit_config::ANGACC_Z_KP, unit_config::ANGACC_Z_KI, unit_config::ANGACC_Z_KD) {
}

actuator_setpoint_t AngularAccelerationController::run(const attitude_estimate_t& estimate, const angular_acceleration_setpoint_t& input) {
  // Limit to maximum angular accelerations
  float rollAccSp = std::max(-unit_config::MAX_PITCH_ROLL_ACC, std::min(unit_config::MAX_PITCH_ROLL_ACC, input.roll_acc_sp));
  float pitchAccSp = std::max(-unit_config::MAX_PITCH_ROLL_ACC, std::min(unit_config::MAX_PITCH_ROLL_ACC, input.pitch_acc_sp));

  // Run PID controllers
  float rollActuatorSp = rollAccPid.calculate(rollAccSp, estimate.roll_acc, unit_config::DT);
  float pitchActuatorSp = pitchAccPid.calculate(pitchAccSp, estimate.pitch_acc, unit_config::DT);
  float yawActuatorSp = yawAccPid.calculate(input.yaw_acc_sp, estimate.yaw_acc, unit_config::DT);

  // Output
  actuator_setpoint_t setpoint {
    .roll_sp = rollActuatorSp,
    .pitch_sp = pitchActuatorSp,
    .yaw_sp = yawActuatorSp,
    .throttle_sp = input.throttle_sp
  };

  return setpoint;
}
