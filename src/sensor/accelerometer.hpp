#ifndef ACCELEROMETER_HPP_
#define ACCELEROMETER_HPP_

#include <array>

#include "sensor/sensor.hpp"

struct AccelerometerReading {
  std::array<float, 3> axes;
};

class Accelerometer : public Sensor {
public:
  using Sensor::Sensor;
  virtual void init() = 0;
  virtual AccelerometerReading readAccel() = 0;
};

#endif // ACCELEROMETER_HPP_
