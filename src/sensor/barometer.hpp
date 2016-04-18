#ifndef BAROMETER_HPP_
#define BAROMETER_HPP_

#include <array>

#include "sensor/sensor.hpp"

struct BarometerReading {
  float pressure;
  float temperature;
};

class Barometer : public Sensor<> {
public:
  virtual void init() = 0;
  virtual BarometerReading readBar() = 0;
};

#endif // BAROMETER_HPP_
