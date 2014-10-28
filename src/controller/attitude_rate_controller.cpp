#include <controller/attitude_rate_controller.hpp>

#include <config.hpp>
#include <controller/controller.hpp>
#include <input/input_source.hpp>

AttitudeRateController::AttitudeRateController()
  : pitchRatePid(10.0, 0.0, 0.0),
    rollRatePid(10.0, 0.0, 0.0),
    yawRatePid(10.0, 0.0, 0.0) {
}

controller_output_t AttitudeRateController::run(const attitude_estimate_t& estimate, const controller_output_t& input) {
  float pitchRateSp = pitchRatePid.calculate(input.setpoints[0], estimate.roll_vel, DT);
  float rollRateSp = rollRatePid.calculate(input.setpoints[1], estimate.pitch_vel, DT);
  float yawRateSp = yawRatePid.calculate(input.setpoints[2], estimate.yaw_vel, DT);

  controller_output_t output = {
    .setpoints = {
      pitchRateSp,
      rollRateSp,
      yawRateSp,
      input.setpoints[3] // pass through throttle
    }
  };

  return output;
}
