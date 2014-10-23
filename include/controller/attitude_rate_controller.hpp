#ifndef ATTITUDE_RATE_CONTROLLER_HPP_
#define ATTITUDE_RATE_CONTROLLER_HPP_

#include <controller/controller.hpp>
#include <controller/pid_controller.hpp>
#include <estimator/attitude_estimator.hpp>

class AttitudeRateController : public Controller {
public:
  AttitudeRateController();

  struct controller_output_t run(struct attitude_estimate_t& estimate, struct controller_output_t& input);

private:
  PIDController pitchRatePid;
  PIDController rollRatePid;
  PIDController yawRatePid;
};

#endif
