#ifndef H3LIS331DL_HPP_
#define H3LIS331DL_HPP_

#include <cstdint>

#include "hal.h"

#include "drivers/spi_device.hpp"
#include "sensor/accelerometer.hpp"

// TODO(yoos): Inherit magnetometer
class H3LIS331DL : protected SPIDevice<8, 8>, public Accelerometer {
public:
  using SPIDevice::SPIDevice;

  void init() override;
  AccelerometerReading readAccel() override;
  bool healthy();

private:
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t val);
};

#endif
