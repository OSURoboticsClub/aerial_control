#ifndef ROCKET_ANGULAR_ACCELERATION_CONTROLLER_HPP_
#define ROCKET_ANGULAR_ACCELERATION_CONTROLLER_HPP_

#include "controller/controller.hpp"
#include "controller/pid_controller.hpp"
#include "controller/setpoint_types.hpp"
#include "estimator/attitude_estimator.hpp"
#include "params/parameter_repository.hpp"

class RocketAngularAccelerationController : public Controller<AngularAccelerationSetpoint, ActuatorSetpoint> {
public:
  static constexpr char const *PARAM_MAX_PITCH_ROLL_ACC = "rocket_angular_acceleration_controller.max_pitch_roll_acc";

  RocketAngularAccelerationController(ParameterRepository& params);

  ActuatorSetpoint run(const WorldEstimate& est, const AngularAccelerationSetpoint& input) override;

private:
  ParameterRepository& params;

  PIDController rollAccPid;
  PIDController pitchAccPid;
  PIDController yawAccPid;
};

#endif
