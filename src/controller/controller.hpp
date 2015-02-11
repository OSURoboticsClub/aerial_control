#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include "estimator/world_estimator.hpp"

/**
 * A single stage in a pipeline of controllers accepting setpoints of type `I`
 * and producing setpoints of type `O`.
 */
template <typename I, typename O>
class Controller {
public:
  virtual O run(const WorldEstimate& world, const I& input) = 0;
};

#endif
