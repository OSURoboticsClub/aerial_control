#include "drivers/lsm303dlhc.hpp"

#include <array>

#include "unit_config.hpp"

void LSM303DLHC::init() {
  // Wake up device and enable X, Y, and Z outputs.
  writeRegister(lsm303dlhc::I2C_AD_CTRL_REG1_A, (1 << 7) | (1 << 4) | (1 << 2) | (1 << 1) | (1 << 0));
}

AccelerometerReading LSM303DLHC::readAccel() {
  txbuf[0] = lsm303dlhc::I2C_AD_OUT_X_L_A | 0x80;

  exchange(1, 6);

  // Swapped for board orientation
  std::array<std::int16_t, 3> raw;
  raw[0] = (rxbuf[1] << 8) | rxbuf[0];
  raw[1] = (rxbuf[3] << 8) | rxbuf[2];
  raw[2] = (rxbuf[5] << 8) | rxbuf[4];

  AccelerometerReading reading;

  for(std::size_t i = 0; i < 3; i++) {
    reading.axes[i] = (float) raw[i] / 32768.0 * 2.0;
  }

  reading.axes[0] += unit_config::ACC_X_OFFSET;
  reading.axes[1] += unit_config::ACC_Y_OFFSET;
  reading.axes[2] += unit_config::ACC_Z_OFFSET;

  return reading;
}

uint8_t LSM303DLHC::readRegister(uint8_t reg) {
  txbuf[0] = reg;

  exchange(1, 1);

  return rxbuf[0];
}

void LSM303DLHC::writeRegister(uint8_t reg, uint8_t val) {
  txbuf[0] = reg;
  txbuf[1] = val;

  exchange(2, 0);
}

bool LSM303DLHC::healthy() {
  return true;
}
