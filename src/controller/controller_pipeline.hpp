#ifndef CONTROLLER_PIPELINE_HPP_
#define CONTROLLER_PIPELINE_HPP_

#include <controller/controller.hpp>
#include <controller/setpoint_types.hpp>

template <typename R>
class ControllerPipeline {
public:
  template <typename SP, typename C>
  R run(attitude_estimate_t& estimate, SP& input, C& tail);

  template <typename SP, typename C, typename... Cs>
  R run(attitude_estimate_t& estimate, SP& input, C& head, Cs&... controllers);
};

#include <controller/controller_pipeline.tpp>

#endif
