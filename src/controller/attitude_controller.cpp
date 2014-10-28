#include <controller/attitude_controller.hpp>

#include <algorithm>

#include <config.hpp>
#include <input/input_source.hpp>

AttitudeController::AttitudeController()
  : pitchPosPid(10.0, 0.0, 0.0),
    rollPosPid(10.0, 0.0, 0.0),
    yawPosPid(10.0, 0.0, 0.0),
    pitchVelPid(10.0, 0.0, 0.0),
    rollVelPid(10.0, 0.0, 0.0),
    yawVelPid(10.0, 0.0, 0.0) {
}

controller_output_t AttitudeController::run(const attitude_estimate_t& estimate, const controller_output_t& input) {
  float pitchPosSp = pitchPosPid.calculate(input.setpoints[0], estimate.roll, DT);
  float rollPosSp = rollPosPid.calculate(input.setpoints[1], estimate.pitch, DT);
  float yawPosSp = yawPosPid.calculate(input.setpoints[2], estimate.yaw, DT);

  // Limit to maximum angles
  pitchPosSp = std::max(-MAX_PITCH_ROLL_POS, std::min(MAX_PITCH_ROLL_POS, pitchPosSp));
  rollPosSp = std::max(-MAX_PITCH_ROLL_POS, std::min(MAX_PITCH_ROLL_POS, rollPosSp));

  float pitchVelSp = pitchVelPid.calculate(pitchPosSp, estimate.roll_vel, DT);
  float rollVelSp = rollVelPid.calculate(rollPosSp, estimate.pitch_vel, DT);
  float yawVelSp = yawVelPid.calculate(yawPosSp, estimate.yaw_vel, DT);

  // Limit to maximum angular rates
  pitchVelSp = std::max(-MAX_PITCH_ROLL_VEL, std::min(MAX_PITCH_ROLL_VEL, pitchVelSp));
  rollVelSp = std::max(-MAX_PITCH_ROLL_VEL, std::min(MAX_PITCH_ROLL_VEL, rollVelSp));

  controller_output_t output = {
    .setpoints = {
      pitchVelSp,
      rollVelSp,
      yawVelSp,
      input.setpoints[3] // pass through throttle
    }
  };

  return output;
}
