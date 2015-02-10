#ifndef THERMISTOR_HPP_
#define THERMISTOR_HPP_

#include <array>

struct ThermistorReading {
  float celsius;
};

class Thermistor {
public:
  virtual void init() = 0;
  virtual ThermistorReading readTherm() = 0;
};

#endif
