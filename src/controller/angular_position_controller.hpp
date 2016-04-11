#ifndef ANGULAR_POSITION_CONTROLLER_HPP_
#define ANGULAR_POSITION_CONTROLLER_HPP_

#include "controller/controller.hpp"
#include "controller/pid_controller.hpp"
#include "controller/setpoint_types.hpp"
#include "estimator/attitude_estimator.hpp"
#include "params/parameter_repository.hpp"

class AngularPositionController : public Controller<AngularPositionSetpoint, AngularVelocitySetpoint> {
public:
  static constexpr char const *PARAM_PID_ROLL_KP = "angular_position_controller.pid_roll_kp";
  static constexpr char const *PARAM_PID_ROLL_KI = "angular_position_controller.pid_roll_ki";
  static constexpr char const *PARAM_PID_ROLL_KD = "angular_position_controller.pid_roll_kd";
  static constexpr char const *PARAM_PID_PITCH_KP = "angular_position_controller.pid_roll_kp";
  static constexpr char const *PARAM_PID_PITCH_KI = "angular_position_controller.pid_roll_ki";
  static constexpr char const *PARAM_PID_PITCH_KD = "angular_position_controller.pid_roll_kd";
  static constexpr char const *PARAM_PID_YAW_KP = "angular_position_controller.pid_roll_kp";
  static constexpr char const *PARAM_PID_YAW_KI = "angular_position_controller.pid_roll_ki";
  static constexpr char const *PARAM_PID_YAW_KD = "angular_position_controller.pid_roll_kd";
  static constexpr char const *PARAM_MAX_PITCH_ROLL_POS = "angular_position_controller.max_pitch_roll_pos";

  AngularPositionController(ParameterRepository& params);

  AngularVelocitySetpoint run(const WorldEstimate& world, const AngularPositionSetpoint& input) override;

private:
  ParameterRepository& params;

  PIDController rollPosPid;
  PIDController pitchPosPid;
  PIDController yawPosPid;
};

#endif
