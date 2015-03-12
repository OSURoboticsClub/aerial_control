#ifndef MS5611_HPP_
#define MS5611_HPP_

#include <cstdint>

#include "hal.h"

#include "drivers/spi_device.hpp"
#include "sensor/barometer.hpp"

// TODO(yoos): Inherit magnetometer
class MS5611 : protected SPIDevice<8, 8>, public Barometer {
public:
  using SPIDevice::SPIDevice;

  void init() override;
  BarometerReading readBar() override;

private:
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t val);
};

#endif
