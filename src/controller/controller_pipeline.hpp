#ifndef CONTROLLER_PIPELINE_HPP_
#define CONTROLLER_PIPELINE_HPP_

#include <controller/controller.hpp>
#include <controller/setpoint_types.hpp>

/**
 * Provides a simple interface to submitting setpoints to a series of
 * controllers to be run sequentially. Produces a final setpoint of type `R`.
 */
template <typename R>
class ControllerPipeline {
public:
  template <typename SP, typename C>
  R run(const attitude_estimate_t& estimate, const SP& input, C& tail);

  template <typename SP, typename C, typename... Cs>
  R run(const attitude_estimate_t& estimate, const SP& input, C& head, Cs&... controllers);
};

#include <controller/controller_pipeline.tpp>

#endif
