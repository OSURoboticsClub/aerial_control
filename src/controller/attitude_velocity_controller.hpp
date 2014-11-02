#ifndef ATTITUDE_VELOCITY_CONTROLLER_HPP_
#define ATTITUDE_VELOCITY_CONTROLLER_HPP_

#include <controller/controller.hpp>
#include <controller/pid_controller.hpp>
#include <controller/setpoint_types.hpp>
#include <estimator/attitude_estimator.hpp>

class AttitudeVelocityController : public Controller<attitude_velocity_setpoint_t, actuator_setpoint_t> {
public:
  AttitudeVelocityController();

  actuator_setpoint_t run(const attitude_estimate_t& estimate, const attitude_velocity_setpoint_t& input) override;

private:
  PIDController pitchVelPid;
  PIDController rollVelPid;
  PIDController yawVelPid;
};

#endif
