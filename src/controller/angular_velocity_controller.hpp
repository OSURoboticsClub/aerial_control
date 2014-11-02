#ifndef ANGULAR_VELOCITY_CONTROLLER_HPP_
#define ANGULAR_VELOCITY_CONTROLLER_HPP_

#include <controller/controller.hpp>
#include <controller/pid_controller.hpp>
#include <controller/setpoint_types.hpp>
#include <estimator/attitude_estimator.hpp>

class AngularVelocityController : public Controller<angular_velocity_setpoint_t, actuator_setpoint_t> {
public:
  AngularVelocityController();

  actuator_setpoint_t run(const attitude_estimate_t& estimate, const angular_velocity_setpoint_t& input) override;

private:
  PIDController pitchVelPid;
  PIDController rollVelPid;
  PIDController yawVelPid;
};

#endif
