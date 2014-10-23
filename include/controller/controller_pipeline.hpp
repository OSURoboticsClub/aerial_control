#ifndef CONTROLLER_PIPELINE_HPP_
#define CONTROLLER_PIPELINE_HPP_

#include <stdint.h>

#include <controller/controller.hpp>

class ControllerPipeline {
public:
  ControllerPipeline(Controller *controllers[], uint8_t size);

  struct controller_output_t run(struct attitude_estimate_t& estimate, struct controller_output_t& input);

private:
  Controller **controllers;
  uint8_t size;
};

#endif
