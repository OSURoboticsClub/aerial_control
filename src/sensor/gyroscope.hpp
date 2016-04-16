#ifndef GYROSCOPE_HPP_
#define GYROSCOPE_HPP_

#include <array>

#include "sensor/sensor.hpp"

struct GyroscopeReading {
  std::array<float, 3> axes;
};

class Gyroscope : public Sensor {
public:
  using Sensor::Sensor;
  virtual void init() = 0;
  virtual GyroscopeReading readGyro() = 0;

  virtual void calibrateStep();
  virtual bool calibrated();
};

#endif
