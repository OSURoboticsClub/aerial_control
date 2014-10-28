#ifndef ATTITUDE_RATE_CONTROLLER_HPP_
#define ATTITUDE_RATE_CONTROLLER_HPP_

#include <controller/controller.hpp>
#include <controller/pid_controller.hpp>
#include <estimator/attitude_estimator.hpp>

class AttitudeRateController : public Controller {
public:
  AttitudeRateController();

  controller_output_t run(const attitude_estimate_t& estimate, const controller_output_t& input);

private:
  PIDController pitchRatePid;
  PIDController rollRatePid;
  PIDController yawRatePid;
};

#endif
