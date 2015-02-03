#include "lsm303dlhc_mag.hpp"

#include <array>

#include "unit_config.hpp"

void LSM303DLHCMag::init() {
  // Enable the magnetometer
  writeRegister(lsm303dlhc_mag::I2C_MR_REG_M, 0x00);

  // Set gain
  writeRegister(lsm303dlhc_mag::I2C_CRB_REG_M, 0x20);
}

magnetometer_reading_t LSM303DLHCMag::readMag() {
  txbuf[0] = lsm303dlhc_mag::I2C_OUT_X_H_M;

  exchange(1, 6);

  // Swapped for board orientation
  std::array<std::int16_t, 3> raw;
  raw[0] = ((rxbuf[1] << 8) | rxbuf[0]) / 1100.0f;
  raw[1] = ((rxbuf[3] << 8) | rxbuf[2]) / 1100.0f;
  raw[2] = ((rxbuf[5] << 8) | rxbuf[4]) / 980.0f;

  magnetometer_reading_t reading;

  for(std::size_t i = 0; i < 3; i++) {
    // TODO(kyle): scale?
    reading.axes[i] = (float) raw[i];
  }

  return reading;
}

uint8_t LSM303DLHCMag::readRegister(uint8_t reg) {
  txbuf[0] = reg;

  exchange(1, 1);

  return rxbuf[0];
}

void LSM303DLHCMag::writeRegister(uint8_t reg, uint8_t val) {
  txbuf[0] = reg;
  txbuf[1] = val;

  exchange(2, 0);
}
