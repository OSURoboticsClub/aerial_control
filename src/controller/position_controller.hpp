#ifndef POSITION_CONTROLLER_HPP_
#define POSITION_CONTROLLER_HPP_

#include "controller/controller.hpp"
#include "controller/pid_controller.hpp"
#include "controller/setpoint_types.hpp"
#include "estimator/attitude_estimator.hpp"

class PositionController : public Controller<position_setpoint_t, angular_position_setpoint_t> {
public:
  PositionController();

  angular_position_setpoint_t run(const attitude_estimate_t& estimate, const position_setpoint_t& input) override;

private:
  PIDController latPid;
  PIDController lonPid;
  PIDController altitudePid;
};

#endif
