#ifndef I2C_CONFIG_HPP_
#define I2C_CONFIG_HPP_

// LSM303DHLC I2C configuration
const I2CConfig lsm303dlhc_i2c_config = {
  OPMODE_I2C,
  400000,
  FAST_DUTY_CYCLE_2
};

void i2cPlatformInit(void);

#endif // I2C_CONFIG_HPP_
