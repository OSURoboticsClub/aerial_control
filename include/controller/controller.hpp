#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <estimator/attitude_estimator.hpp>

struct controller_output_t {
  float setpoints[4];
};

class Controller {
public:
  virtual controller_output_t run(const attitude_estimate_t& estimate, const controller_output_t& input) =0;

  bool isPassthrough() const;
  void setPassthrough(bool passthrough);

private:
  bool passthrough;
};

#endif
