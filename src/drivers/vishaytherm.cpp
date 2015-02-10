#include "drivers/vishaytherm.hpp"

#include "unit_config.hpp"

void VishayTherm::init() {
}

ThermistorReading VishayTherm::readTherm() {
  ThermistorReading reading;

  reading.celsius = 2.345f;

  return reading;
}
