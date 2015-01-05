#include "variant/i2c_platform.hpp"

#include "hal.h"

// LSM303DHLC I2C configuration
static const I2CConfig lsm303dlhc_i2c_config = {
  0x00902025, // voodoo magic
  0,
  0
};

I2CPlatform::I2CPlatform() {
  // TODO: should i2cStart be in an i2c_device.cpp file?
  i2cStart(&I2CD1, &lsm303dlhc_i2c_config);
}
