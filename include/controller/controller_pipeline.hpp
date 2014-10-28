#ifndef CONTROLLER_PIPELINE_HPP_
#define CONTROLLER_PIPELINE_HPP_

#include <cstdint>

#include <controller/controller.hpp>

class ControllerPipeline {
public:
  ControllerPipeline(Controller *controllers[], std::uint8_t size);

  controller_output_t run(attitude_estimate_t& estimate, controller_output_t& input);

private:
  Controller **controllers;
  std::uint8_t size;
};

#endif
