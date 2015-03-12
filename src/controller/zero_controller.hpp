#ifndef ZERO_CONTROLLER_HPP_
#define ZERO_CONTROLLER_HPP_

#include "controller/controller.hpp"
#include "controller/setpoint_types.hpp"
#include "estimator/attitude_estimator.hpp"

/**
 * A dumb controller that simply outputs zeros on all channels.
 */
template <typename SP>
class ZeroController : public Controller<SP, ActuatorSetpoint> {
public:
  ZeroController();

  ActuatorSetpoint run(const WorldEstimate& world, const SP& input) override;
};

#include "controller/zero_controller.tpp"

#endif
