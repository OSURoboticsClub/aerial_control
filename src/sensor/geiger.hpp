#ifndef GEIGER_HPP_
#define GEIGER_HPP_

#include <array>

#include "sensor/sensor.hpp"

struct GeigerReading {
  uint8_t blips;
};

class Geiger : public Sensor<> {
public:
  virtual void init() = 0;
  virtual GeigerReading readGeiger() = 0;
};

#endif
