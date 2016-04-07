#ifndef ANGULAR_VELOCITY_CONTROLLER_HPP_
#define ANGULAR_VELOCITY_CONTROLLER_HPP_

#include "controller/controller.hpp"
#include "controller/pid_controller.hpp"
#include "controller/setpoint_types.hpp"
#include "estimator/attitude_estimator.hpp"
#include "params/parameter_repository.hpp"

class AngularVelocityController : public Controller<AngularVelocitySetpoint, AngularAccelerationSetpoint> {
public:
  static constexpr char const *PARAM_PID_ROLL_KP = "angular_velocity_controller.pid_roll_kp";
  static constexpr char const *PARAM_PID_ROLL_KI = "angular_velocity_controller.pid_roll_ki";
  static constexpr char const *PARAM_PID_ROLL_KD = "angular_velocity_controller.pid_roll_kd";
  static constexpr char const *PARAM_PID_PITCH_KP = "angular_velocity_controller.pid_roll_kp";
  static constexpr char const *PARAM_PID_PITCH_KI = "angular_velocity_controller.pid_roll_ki";
  static constexpr char const *PARAM_PID_PITCH_KD = "angular_velocity_controller.pid_roll_kd";
  static constexpr char const *PARAM_PID_YAW_KP = "angular_velocity_controller.pid_roll_kp";
  static constexpr char const *PARAM_PID_YAW_KI = "angular_velocity_controller.pid_roll_ki";
  static constexpr char const *PARAM_PID_YAW_KD = "angular_velocity_controller.pid_roll_kd";
  static constexpr char const *PARAM_MAX_PITCH_ROLL_VEL = "angular_velocity_controller.max_pitch_roll_vel";

  AngularVelocityController(ParameterRepository& params);

  AngularAccelerationSetpoint run(const WorldEstimate& world, const AngularVelocitySetpoint& input) override;

private:
  ParameterRepository& params;

  PIDController rollVelPid;
  PIDController pitchVelPid;
  PIDController yawVelPid;
};

#endif
