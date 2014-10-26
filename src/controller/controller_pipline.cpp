#include <controller/controller_pipeline.hpp>

ControllerPipeline::ControllerPipeline(Controller *controllers[], uint8_t size)
  : controllers(controllers),
    size(size) {
}

struct controller_output_t ControllerPipeline::run(struct attitude_estimate_t& estimate, struct controller_output_t& input) {
  struct controller_output_t result = input;

  // Run all controllers that are not marked passthrough
  for(int i = 0; i < size; i++) {
    if(!controllers[i]->isPassthrough()) {
      result = controllers[i]->run(estimate, result);
    }
  }

  return result;
}
