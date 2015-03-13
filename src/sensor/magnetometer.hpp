#ifndef MAGNETOMETER_HPP_
#define MAGNETOMETER_HPP_

#include <array>

struct MagnetometerReading {
  std::array<float, 3> axes;
};

class Magnetometer {
public:
  virtual void init() = 0;
  virtual bool isHealthy() = 0;
  virtual MagnetometerReading readMag() = 0;
};

#endif
