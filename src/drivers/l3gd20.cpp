#include "drivers/l3gd20.hpp"

#include <array>

#include "unit_config.hpp"

void L3GD20::init() {
  // Wake up device, enable X, Y, and Z outputs, and set 760Hz mode.
  writeRegister(l3gd20::SPI_AD_CTRL_REG1, 0x0F | (1 << 7) | (1 << 6));

  // Set 2000 DPS mode
  writeRegister(l3gd20::SPI_AD_CTRL_REG4, (1 << 5) | (1 << 4));
}

bool L3GD20::isHealthy() {
  return false;   // TODO
}

GyroscopeReading L3GD20::readGyro() {
  txbuf[0] = l3gd20::SPI_RW | l3gd20::SPI_MS | l3gd20::SPI_AD_OUT_X_L;
  txbuf[1] = 0xFF;
  txbuf[2] = 0xFF;

  exchange(7);

  // Swapped for board orientation
  std::array<int16_t, 3> raw;
  raw[0] = -((rxbuf[4] << 8) | rxbuf[3]);
  raw[1] = (rxbuf[2] << 8) | rxbuf[1];
  raw[2] = (rxbuf[6] << 8) | rxbuf[5];

  GyroscopeReading reading;

  for(std::size_t i = 0; i < 3; i++) {
    reading.axes[i] = (float) raw[i] * l3gd20::SENSITIVITY_2000DPS * l3gd20::DPS_TO_RADS;
  }

  reading.axes[0] += unit_config::GYR_X_OFFSET;
  reading.axes[1] += unit_config::GYR_Y_OFFSET;
  reading.axes[2] += unit_config::GYR_Z_OFFSET;

  return reading;
}

uint8_t L3GD20::readRegister(uint8_t reg) {
  txbuf[0] = l3gd20::SPI_RW | reg;
  txbuf[1] = 0xFF;

  exchange(2);

  return rxbuf[1];
}

void L3GD20::writeRegister(uint8_t reg, uint8_t val) {
  txbuf[0] = reg;
  txbuf[1] = val;

  exchange(2);
}
