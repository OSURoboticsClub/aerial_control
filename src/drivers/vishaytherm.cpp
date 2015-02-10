#include "drivers/vishaytherm.hpp"

#include "chprintf.h"
#include "unit_config.hpp"

void VishayTherm::init() {
}

ThermistorReading VishayTherm::readTherm() {
  ThermistorReading reading;

  reading.celsius = 2.345f;

  chprintf((BaseSequentialStream*)&SD4, "%f\r\n", reading.celsius);

  return reading;
}
