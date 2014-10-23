#ifndef ACCELEROMETER_HPP_
#define ACCELEROMETER_HPP_

#include <ch.hpp>

struct accelerometer_reading_t {
  float axes[3];
};

class Accelerometer {
public:
  virtual void init() =0;
  virtual struct accelerometer_reading_t read() =0;
};

#endif
