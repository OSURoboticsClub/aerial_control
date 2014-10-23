#include <controller/controller_pipeline.hpp>

ControllerPipeline::ControllerPipeline(Controller *controllers[], uint8_t size)
  : controllers(controllers),
    size(size) {
}

struct controller_output_t ControllerPipeline::run(struct attitude_estimate_t& estimate, struct controller_output_t& input) {
  struct controller_output_t result = controllers[0]->run(estimate, input);

  for(uint8_t i = 1; i < size; i++) {
    result = controllers[i]->run(estimate, result);
  }

  return result;
}

