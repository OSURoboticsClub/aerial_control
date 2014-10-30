#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <estimator/attitude_estimator.hpp>

template <typename I, typename O>
class Controller {
public:
  virtual O run(const attitude_estimate_t& estimate, const I& input) =0;
};

#endif
