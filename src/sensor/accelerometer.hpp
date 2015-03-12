#ifndef ACCELEROMETER_HPP_
#define ACCELEROMETER_HPP_

#include <array>

struct AccelerometerReading {
  std::array<float, 3> axes;
};

class Accelerometer {
public:
  virtual void init() = 0;
  virtual AccelerometerReading readAccel() = 0;
};

#endif
