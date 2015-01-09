#ifndef GYROSCOPE_HPP_
#define GYROSCOPE_HPP_

#include <array>

struct gyroscope_reading_t {
  std::array<float, 3> axes;
};

class Gyroscope {
public:
  virtual void init() = 0;
  virtual gyroscope_reading_t readGyro() = 0;
};

#endif
