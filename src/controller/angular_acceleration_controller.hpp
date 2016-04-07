#ifndef ANGULAR_ACCELERATION_CONTROLLER_HPP_
#define ANGULAR_ACCELERATION_CONTROLLER_HPP_

#include "controller/controller.hpp"
#include "controller/pid_controller.hpp"
#include "controller/setpoint_types.hpp"
#include "estimator/attitude_estimator.hpp"
#include "params/parameter_repository.hpp"

class AngularAccelerationController : public Controller<AngularAccelerationSetpoint, ActuatorSetpoint> {
public:
  static constexpr char const *PARAM_PID_ROLL_KP = "angular_acceleration_controller.pid_roll_kp";
  static constexpr char const *PARAM_PID_ROLL_KI = "angular_acceleration_controller.pid_roll_ki";
  static constexpr char const *PARAM_PID_ROLL_KD = "angular_acceleration_controller.pid_roll_kd";
  static constexpr char const *PARAM_PID_PITCH_KP = "angular_acceleration_controller.pid_roll_kp";
  static constexpr char const *PARAM_PID_PITCH_KI = "angular_acceleration_controller.pid_roll_ki";
  static constexpr char const *PARAM_PID_PITCH_KD = "angular_acceleration_controller.pid_roll_kd";
  static constexpr char const *PARAM_PID_YAW_KP = "angular_acceleration_controller.pid_roll_kp";
  static constexpr char const *PARAM_PID_YAW_KI = "angular_acceleration_controller.pid_roll_ki";
  static constexpr char const *PARAM_PID_YAW_KD = "angular_acceleration_controller.pid_roll_kd";
  static constexpr char const *PARAM_MAX_PITCH_ROLL_ACC = "angular_acceleration_controller.max_pitch_roll_acc";

  AngularAccelerationController(ParameterRepository& params);

  ActuatorSetpoint run(const WorldEstimate& world, const AngularAccelerationSetpoint& input) override;

private:
  ParameterRepository& params;

  PIDController rollAccPid;
  PIDController pitchAccPid;
  PIDController yawAccPid;
};

#endif
