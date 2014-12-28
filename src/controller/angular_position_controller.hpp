#ifndef ANGULAR_POSITION_CONTROLLER_HPP_
#define ANGULAR_POSITION_CONTROLLER_HPP_

#include "hal_config.hpp"
#include "controller/controller.hpp"
#include "controller/pid_controller.hpp"
#include "controller/setpoint_types.hpp"
#include "estimator/attitude_estimator.hpp"

class AngularPositionController : public Controller<angular_position_setpoint_t, angular_velocity_setpoint_t> {
public:
  AngularPositionController();

  angular_velocity_setpoint_t run(const attitude_estimate_t& estimate, const angular_position_setpoint_t& input) override;

private:
  PIDController rollPosPid;
  PIDController pitchPosPid;
  PIDController yawPosPid;
};

#endif
