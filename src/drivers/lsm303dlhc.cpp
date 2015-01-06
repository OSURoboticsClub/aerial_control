#include "drivers/lsm303dlhc.hpp"

#include <algorithm>
#include <array>
#include <cstddef>

void LSM303DLHC::init() {
  // Wake up device and enable X, Y, and Z outputs.
  writeRegister(LSM303_I2C_AD_CTRL_REG1_A, (1 << 7) | (1 << 4) | (1 << 2) | (1 << 1) | (1 << 0));
}

accelerometer_reading_t LSM303DLHC::readAccel() {
  std::fill(std::begin(txbuf), std::end(txbuf), 0);
  txbuf[0] = LSM303_I2C_AD_OUT_X_L_A | 0x80;

  exchange();

  // Swapped for board orientation
  std::array<std::int16_t, 3> raw;
  raw[0] = -((rxbuf[3] << 8) | rxbuf[2]);
  raw[1] = (rxbuf[1] << 8) | rxbuf[0];
  raw[2] = (rxbuf[5] << 8) | rxbuf[4];

  accelerometer_reading_t reading;

  for(std::size_t i = 0; i < 3; i++) {
    // TODO: Scale to m/s^2
    reading.axes[i] = (float) raw[i];
  }

  return reading;
}

uint8_t LSM303DLHC::readRegister(uint8_t reg) {
  std::fill(std::begin(txbuf), std::end(txbuf), 0);
  txbuf[0] = reg;

  exchange();

  return rxbuf[0];
}

void LSM303DLHC::writeRegister(uint8_t reg, uint8_t val) {
  std::fill(std::begin(txbuf), std::end(txbuf), 0);
  txbuf[0] = reg;
  txbuf[1] = val;

  exchange();
}
