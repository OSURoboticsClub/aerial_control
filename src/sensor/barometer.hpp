#ifndef BAROMETER_HPP_
#define BAROMETER_HPP_

#include <array>

struct BarometerReading {
  float pressure;
  float temperature;
};

class Barometer {
public:
  virtual void init() = 0;
  virtual bool isHealthy() = 0;
  virtual BarometerReading readBar() = 0;
};

#endif
