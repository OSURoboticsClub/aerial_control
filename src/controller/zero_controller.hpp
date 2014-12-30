#ifndef ZERO_CONTROLLER_HPP_
#define ZERO_CONTROLLER_HPP_

#include "controller/controller.hpp"
#include "controller/setpoint_types.hpp"
#include "estimator/attitude_estimator.hpp"

template <typename SP>
class ZeroController : public Controller<SP, actuator_setpoint_t> {
public:
  ZeroController();

  actuator_setpoint_t run(const attitude_estimate_t& estimate, const SP& input) override;
};

#include "controller/zero_controller.tpp"

#endif
