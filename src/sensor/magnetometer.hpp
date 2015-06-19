#ifndef MAGNETOMETER_HPP_
#define MAGNETOMETER_HPP_

#include <array>

#include "sensor/sensor.hpp"

struct MagnetometerReading {
  std::array<float, 3> axes;
};

class Magnetometer : public Sensor {
public:
  virtual void init() = 0;
  virtual MagnetometerReading readMag() = 0;
};

#endif
