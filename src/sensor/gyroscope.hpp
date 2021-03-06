#ifndef GYROSCOPE_HPP_
#define GYROSCOPE_HPP_

#include <array>

#include "sensor/sensor.hpp"

struct GyroscopeReading {
  std::array<float, 3> axes;
};

class Gyroscope : public Sensor<> {
public:
  virtual void init() = 0;
  virtual GyroscopeReading readGyro() = 0;

  virtual void calibrateStep();
  virtual bool calibrated() const;

private:
  unsigned int calibrationCount = 0;
};

#endif // GYROSCOPE_HPP_
