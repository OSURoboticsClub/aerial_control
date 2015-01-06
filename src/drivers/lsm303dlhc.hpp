#ifndef LSM303DLHC_HPP_
#define LSM303DLHC_HPP_

#include "hal.h"

#include "drivers/i2c_device.hpp"
#include "sensor/accelerometer.hpp"

#define LSM303_I2C_AD_CTRL_REG1_A     0x20
#define LSM303_I2C_AD_CTRL_REG2_A     0x21
#define LSM303_I2C_AD_CTRL_REG3_A     0x22
#define LSM303_I2C_AD_CTRL_REG4_A     0x23
#define LSM303_I2C_AD_CTRL_REG5_A     0x24
#define LSM303_I2C_AD_CTRL_REG6_A     0x25
#define LSM303_I2C_AD_REFERENCE_A     0x26
#define LSM303_I2C_AD_STATUS_REG_A    0x27
#define LSM303_I2C_AD_OUT_X_L_A       0x28
#define LSM303_I2C_AD_OUT_X_H_A       0x29
#define LSM303_I2C_AD_OUT_Y_L_A       0x2A
#define LSM303_I2C_AD_OUT_Y_H_A       0x2B
#define LSM303_I2C_AD_OUT_Z_L_A       0x2C
#define LSM303_I2C_AD_OUT_Z_H_A       0x2D
#define LSM303_I2C_AD_FIFO_CTRL_REG_A 0x2E
#define LSM303_I2C_AD_FIFO_SRC_REG_A  0x2F
#define LSM303_I2C_AD_INT1_CFG_A      0x30
#define LSM303_I2C_AD_INT1_SOURCE_A   0x31
#define LSM303_I2C_AD_INT1_THS_A      0x32
#define LSM303_I2C_AD_INT1_DURATION_A 0x33
#define LSM303_I2C_AD_INT2_CFG_A      0x34
#define LSM303_I2C_AD_INT2_SOURCE_A   0x35
#define LSM303_I2C_AD_INT2_THS_A      0x36
#define LSM303_I2C_AD_INT2_DURATION_A 0x37
#define LSM303_I2C_AD_CLICK_CFG_A     0x38
#define LSM303_I2C_AD_CLICK_SRC_A     0x39
#define LSM303_I2C_AD_CLICK_THS_A     0x3A
#define LSM303_I2C_AD_TIME_LIMIT_A    0x3B
#define LSM303_I2C_AD_TIME_LATENCY_A  0x3C
#define LSM303_I2C_AD_TIME_WINDOW_A   0x3D

class LSM303DLHC : protected I2CDevice<8, 8>, public Accelerometer {
public:
  using I2CDevice::I2CDevice;

  void init() override;
  accelerometer_reading_t readAccel() override;

private:
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t val);
};

#endif
