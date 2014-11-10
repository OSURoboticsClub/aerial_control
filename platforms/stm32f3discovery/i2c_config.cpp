#include <hal.h>

#include <i2c_config.hpp>

void i2cPlatformInit(void) {
  i2cInit();
  i2cStart(&I2CD1, &lsm303dlhc_i2c_config);
}
