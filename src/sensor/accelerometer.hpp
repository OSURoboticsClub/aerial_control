#ifndef ACCELEROMETER_HPP_
#define ACCELEROMETER_HPP_

#include <array>

struct accelerometer_reading_t {
  std::array<float, 3> axes;
};

class Accelerometer {
public:
  virtual void init() = 0;
  virtual accelerometer_reading_t readAccel() = 0;
};

#endif
