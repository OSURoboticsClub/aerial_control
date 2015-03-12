#ifndef L3GD20_HPP_
#define L3GD20_HPP_

#include <cstdint>

#include "hal.h"

#include "sensor/gyroscope.hpp"
#include "drivers/spi_device.hpp"

namespace l3gd20 {

const float SENSITIVITY_250DPS          = 0.00875F;
const float SENSITIVITY_500DPS          = 0.0175F;
const float SENSITIVITY_2000DPS         = 0.070F;
const float DPS_TO_RADS                 = 0.017453293F;

const std::uint8_t WHO_AM_I             = 0xD4;

const std::uint8_t SPI_MS               = 0x40;
const std::uint8_t SPI_RW               = 0x80;

const std::uint8_t REG_WHO_AM_I      = 0x0F;
const std::uint8_t REG_CTRL_REG1     = 0x20;
const std::uint8_t REG_CTRL_REG2     = 0x21;
const std::uint8_t REG_CTRL_REG3     = 0x22;
const std::uint8_t REG_CTRL_REG4     = 0x23;
const std::uint8_t REG_CTRL_REG5     = 0x24;
const std::uint8_t REG_REFERENCE     = 0x25;
const std::uint8_t REG_OUT_TEMP      = 0x26;
const std::uint8_t REG_STATUS_REG    = 0x27;
const std::uint8_t REG_OUT_X_L       = 0x28;
const std::uint8_t REG_OUT_X_H       = 0x29;
const std::uint8_t REG_OUT_Y_L       = 0x2A;
const std::uint8_t REG_OUT_Y_H       = 0x2B;
const std::uint8_t REG_OUT_Z_L       = 0x2C;
const std::uint8_t REG_OUT_Z_H       = 0x2D;
const std::uint8_t REG_FIFO_CTRL_REG = 0x2E;
const std::uint8_t REG_FIFO_SRC_REG  = 0x2F;
const std::uint8_t REG_INT1_CFG      = 0x30;
const std::uint8_t REG_INT1_SRC      = 0x31;
const std::uint8_t REG_INT1_TSH_XH   = 0x32;
const std::uint8_t REG_INT1_TSH_XL   = 0x33;
const std::uint8_t REG_INT1_TSH_YH   = 0x34;
const std::uint8_t REG_INT1_TSH_YL   = 0x35;
const std::uint8_t REG_INT1_TSH_ZH   = 0x36;
const std::uint8_t REG_INT1_TSH_ZL   = 0x37;
const std::uint8_t REG_INT1_DURATION = 0x38;

}

class L3GD20 : protected SPIDevice<8, 8>, public Gyroscope {
public:
  using SPIDevice::SPIDevice;

  void init() override;
  bool healthy();
  GyroscopeReading readGyro() override;

private:
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t val);
};

#endif
