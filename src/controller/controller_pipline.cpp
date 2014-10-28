#include <controller/controller_pipeline.hpp>

ControllerPipeline::ControllerPipeline(Controller *controllers[], std::uint8_t size)
  : controllers(controllers),
    size(size) {
}

controller_output_t ControllerPipeline::run(attitude_estimate_t& estimate, controller_output_t& input) {
  auto result = input;

  // Run all controllers that are not marked passthrough
  for(int i = 0; i < size; i++) {
    if(!controllers[i]->isPassthrough()) {
      result = controllers[i]->run(estimate, result);
    }
  }

  return result;
}
