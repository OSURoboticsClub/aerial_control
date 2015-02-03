#ifndef MAGNETOMETER_HPP_
#define MAGNETOMETER_HPP_

#include <array>

struct magnetometer_reading_t {
  std::array<float, 3> axes;
};

class Magnetometer {
public:
  virtual void init() = 0;
  virtual magnetometer_reading_t readMag() = 0;
};

#endif
