#ifndef LSM303DLHC_HPP_
#define LSM303DLHC_HPP_

#include <cstdint>

#include "hal.h"

#include "drivers/i2c_device.hpp"
#include "sensor/accelerometer.hpp"

namespace lsm303dlhc {

const std::uint8_t I2C_AD_CTRL_REG1_A     = 0x20;
const std::uint8_t I2C_AD_CTRL_REG2_A     = 0x21;
const std::uint8_t I2C_AD_CTRL_REG3_A     = 0x22;
const std::uint8_t I2C_AD_CTRL_REG4_A     = 0x23;
const std::uint8_t I2C_AD_CTRL_REG5_A     = 0x24;
const std::uint8_t I2C_AD_CTRL_REG6_A     = 0x25;
const std::uint8_t I2C_AD_REFERENCE_A     = 0x26;
const std::uint8_t I2C_AD_STATUS_REG_A    = 0x27;
const std::uint8_t I2C_AD_OUT_X_L_A       = 0x28;
const std::uint8_t I2C_AD_OUT_X_H_A       = 0x29;
const std::uint8_t I2C_AD_OUT_Y_L_A       = 0x2A;
const std::uint8_t I2C_AD_OUT_Y_H_A       = 0x2B;
const std::uint8_t I2C_AD_OUT_Z_L_A       = 0x2C;
const std::uint8_t I2C_AD_OUT_Z_H_A       = 0x2D;
const std::uint8_t I2C_AD_FIFO_CTRL_REG_A = 0x2E;
const std::uint8_t I2C_AD_FIFO_SRC_REG_A  = 0x2F;
const std::uint8_t I2C_AD_INT1_CFG_A      = 0x30;
const std::uint8_t I2C_AD_INT1_SOURCE_A   = 0x31;
const std::uint8_t I2C_AD_INT1_THS_A      = 0x32;
const std::uint8_t I2C_AD_INT1_DURATION_A = 0x33;
const std::uint8_t I2C_AD_INT2_CFG_A      = 0x34;
const std::uint8_t I2C_AD_INT2_SOURCE_A   = 0x35;
const std::uint8_t I2C_AD_INT2_THS_A      = 0x36;
const std::uint8_t I2C_AD_INT2_DURATION_A = 0x37;
const std::uint8_t I2C_AD_CLICK_CFG_A     = 0x38;
const std::uint8_t I2C_AD_CLICK_SRC_A     = 0x39;
const std::uint8_t I2C_AD_CLICK_THS_A     = 0x3A;
const std::uint8_t I2C_AD_TIME_LIMIT_A    = 0x3B;
const std::uint8_t I2C_AD_TIME_LATENCY_A  = 0x3C;
const std::uint8_t I2C_AD_TIME_WINDOW_A   = 0x3D;

}

class LSM303DLHC : protected I2CDevice<8, 8>, public Accelerometer {
public:
  using I2CDevice::I2CDevice;

  void init() override;
  AccelerometerReading readAccel() override;
  bool healthy();

private:
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t val);
};

#endif
