#ifndef ROCKET_ANGULAR_ACCELERATION_CONTROLLER_HPP_
#define ROCKET_ANGULAR_ACCELERATION_CONTROLLER_HPP_

#include "controller/controller.hpp"
#include "controller/pid_controller.hpp"
#include "controller/setpoint_types.hpp"
#include "estimator/attitude_estimator.hpp"

class RocketAngularAccelerationController : public Controller<angular_acceleration_setpoint_t, actuator_setpoint_t> {
public:
  RocketAngularAccelerationController();

  actuator_setpoint_t run(const attitude_estimate_t& estimate, const angular_acceleration_setpoint_t& input) override;

private:
  PIDController rollAccPid;
  PIDController pitchAccPid;
  PIDController yawAccPid;
};

#endif
