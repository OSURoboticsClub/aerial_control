#ifndef GYROSCOPE_HPP_
#define GYROSCOPE_HPP_

#include <array>

#include "sensor/sensor.hpp"

struct GyroscopeReading {
  std::array<float, 3> axes;
};

class Gyroscope : public Sensor {
public:
  Gyroscope();
  virtual void init() = 0;
  virtual GyroscopeReading readGyro() = 0;

  void setGyrOffsets(std::array<float, 3> newOffsets);
  void clearGyrOffsets(void);

protected:
  std::array<float, 3> gyrOffsets;
};

#endif
