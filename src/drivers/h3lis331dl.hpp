#ifndef H3LIS331DL_HPP_
#define H3LIS331DL_HPP_

#include <cstdint>

#include "hal.h"

#include "drivers/spi_device.hpp"
#include "sensor/accelerometer.hpp"

namespace h3lis331dl {

const uint8_t WHO_AM_I        = 0x0F;   // R
const uint8_t CTRL_REG1       = 0x20;   // R/W
const uint8_t CTRL_REG2       = 0x21;   // R/W
const uint8_t CTRL_REG3       = 0x22;   // R/W
const uint8_t CTRL_REG4       = 0x23;   // R/W
const uint8_t CTRL_REG5       = 0x24;   // R/W
const uint8_t HP_FILTER_RESET = 0x25;   // R
const uint8_t REFERENCE       = 0x26;   // R/W
const uint8_t STATUS_REG      = 0x27;   // R
const uint8_t OUT_X_L         = 0x28;   // R
const uint8_t OUT_X_H         = 0x29;   // R
const uint8_t OUT_Y_L         = 0x2A;   // R
const uint8_t OUT_Y_H         = 0x2B;   // R
const uint8_t OUT_Z_L         = 0x2C;   // R
const uint8_t OUT_Z_H         = 0x2D;   // R
const uint8_t INT1_CFG        = 0x30;   // R/W
const uint8_t INT1_SRC        = 0x31;   // R
const uint8_t INT1_THS        = 0x32;   // R/W
const uint8_t INT1_DURATION   = 0x33;   // R/W
const uint8_t INT2_CFG        = 0x34;   // R/W
const uint8_t INT2_SRC        = 0x35;   // R
const uint8_t INT2_THS        = 0x36;   // R/W
const uint8_t INT2_DURATION   = 0x37;   // R/W

}

class H3LIS331DL : protected SPIDevice<8, 8>, public Accelerometer {
public:
  using SPIDevice::SPIDevice;

  void init() override;
  AccelerometerReading readAccel() override;
  bool healthy() override;

private:
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t val);
};

#endif
