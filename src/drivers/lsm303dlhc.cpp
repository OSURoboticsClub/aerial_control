#include <drivers/lsm303dlhc.hpp>

#include <hal.h>

#include <hal_config.hpp>

LSM303DLHC::LSM303DLHC(I2CDriver *i2c) : i2c(i2c) {
}

void LSM303DLHC::init() {
  i2cStart(i2c, &lsm303dlhc_i2c_config);

  // Wake up device and enable X, Y, and Z outputs.
  writeRegister(LSM303_I2C_AD_CTRL_REG1_A, (1 << 7) | (1 << 4) | (1 << 2) | (1 << 1) | (1 << 0));
}

accelerometer_reading_t LSM303DLHC::read() {
  uint8_t txbuf[8];
  uint8_t rxbuf[8];
  int16_t raw[3];

  txbuf[0] = LSM303_I2C_AD_OUT_X_L_A | 0x80;

  i2cAcquireBus(this->i2c);
  i2cMasterTransmitTimeout(this->i2c, LSM303_I2C_ACC_ADDRESS, txbuf, 1, rxbuf, 6, TIME_INFINITE);
  i2cReleaseBus(this->i2c);

  // Swapped for board orientation
  raw[0] = (rxbuf[1] << 8) | rxbuf[0];
  raw[1] = -((rxbuf[3] << 8) | rxbuf[2]);
  raw[2] = (rxbuf[5] << 8) | rxbuf[4];

  accelerometer_reading_t reading;

  for(int i = 0; i < 3; i++) {
    // TODO: Scale to m/s^2
    reading.axes[i] = (float) raw[i];
  }

  return reading;
}

uint8_t LSM303DLHC::readRegister(uint8_t reg) {
  uint8_t txbuf[8];
  uint8_t rxbuf[8];

  txbuf[0] = reg;

  i2cAcquireBus(this->i2c);
  i2cMasterTransmitTimeout(this->i2c, LSM303_I2C_ACC_ADDRESS, txbuf, 1, rxbuf, 1, TIME_INFINITE);
  i2cReleaseBus(this->i2c);

  return rxbuf[0];
}

void LSM303DLHC::writeRegister(uint8_t reg, uint8_t val) {
  uint8_t txbuf[8];
  uint8_t rxbuf[8];

  txbuf[0] = reg;
  txbuf[1] = val;

  i2cAcquireBus(this->i2c);
  i2cMasterTransmitTimeout(this->i2c, LSM303_I2C_ACC_ADDRESS, txbuf, 2, rxbuf, 0, TIME_INFINITE);
  i2cReleaseBus(this->i2c);
}
