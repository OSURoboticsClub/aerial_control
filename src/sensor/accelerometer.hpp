#ifndef ACCELEROMETER_HPP_
#define ACCELEROMETER_HPP_

#include <array>

#include "sensor/sensor.hpp"

struct AccelerometerReading {
  std::array<float, 3> axes;
};

class Accelerometer : public Sensor {
public:
  Accelerometer();
  virtual void init() = 0;
  virtual AccelerometerReading readAccel() = 0;

  void setAccOffsets(std::array<float, 3> newOffsets);
  void clearAccOffsets(void);

protected:
  std::array<float, 3> accOffsets;
};

#endif
