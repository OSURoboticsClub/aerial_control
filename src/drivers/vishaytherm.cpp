#include "drivers/vishaytherm.hpp"

#include "chprintf.h"
#include "unit_config.hpp"

void VishayTherm::init() {
}

ThermistorReading VishayTherm::readTherm() {
  update();   // Update ADC measurements. TODO(yoos): "update" is probably the wrong term to be using.

  ThermistorReading reading;

  //reading.celsius = (*avg_ch)[0] / 1024.0f;

  //chprintf((BaseSequentialStream*)&SD4, "%f\r\n", reading.celsius);

  return reading;
}
