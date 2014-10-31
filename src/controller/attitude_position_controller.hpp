#ifndef ATTITUDE_POSITION_CONTROLLER_HPP_
#define ATTITUDE_POSITION_CONTROLLER_HPP_

#include <config.hpp>
#include <controller/controller.hpp>
#include <controller/pid_controller.hpp>
#include <controller/setpoint_types.hpp>
#include <estimator/attitude_estimator.hpp>

class AttitudePositionController : public Controller<attitude_position_setpoint_t, attitude_velocity_setpoint_t> {
public:
  AttitudePositionController();

  attitude_velocity_setpoint_t run(const attitude_estimate_t& estimate, const attitude_position_setpoint_t& input) override;

private:
  PIDController pitchPosPid;
  PIDController rollPosPid;
  PIDController yawPosPid;
};

#endif
