#ifndef VISHAYTHERM_HPP_
#define VISHAYTHERM_HPP_

#include <cstdint>

#include "hal.h"

#include "drivers/adc_device.hpp"
#include "sensor/thermistor.hpp"

class VishayTherm : protected ADCDevice<2, 4>, public Thermistor {
public:
  using ADCDevice::ADCDevice;

  void init() override;
  ThermistorReading readTherm() override;
};

#endif
