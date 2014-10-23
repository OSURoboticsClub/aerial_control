#ifndef ATTITUDE_CONTROLLER_HPP_
#define ATTITUDE_CONTROLLER_HPP_

#include <controller/controller.hpp>
#include <controller/pid_controller.hpp>
#include <estimator/attitude_estimator.hpp>

class AttitudeController : public Controller {
public:
  AttitudeController();

  struct controller_output_t run(struct attitude_estimate_t& estimate, struct controller_output_t& input);

private:
  PIDController pitchPosPid;
  PIDController rollPosPid;
  PIDController yawPosPid;

  PIDController pitchVelPid;
  PIDController rollVelPid;
  PIDController yawVelPid;
};

#endif
