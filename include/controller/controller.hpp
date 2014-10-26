#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

struct controller_output_t {
  float setpoints[4];
};

class Controller {
public:
  virtual struct controller_output_t run(struct attitude_estimate_t& estimate, struct controller_output_t& input) =0;

  bool isPassthrough();
  void setPassthrough(bool passthrough);

private:
  bool passthrough;
};

#endif
