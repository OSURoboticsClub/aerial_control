#include <drivers/l3gd20.hpp>

#include <hal.h>

#include <hal_config.hpp>

L3GD20::L3GD20(SPIDriver *spid) : spid(spid) {
}

void L3GD20::init() {
  // Wake up device, enable X, Y, and Z outputs, and set 760Hz mode.
  writeRegister(L3GD20_SPI_AD_CTRL_REG1, 0x0F | (1 << 7) | (1 << 6));

  // Set 2000 DPS mode
  writeRegister(L3GD20_SPI_AD_CTRL_REG4, (1 << 5) | (1 << 4));
}

gyroscope_reading_t L3GD20::readGyro() {
  uint8_t txbuf[8];
  uint8_t rxbuf[8];
  int16_t raw[3];

  txbuf[0] = L3GD20_SPI_RW | L3GD20_SPI_MS | L3GD20_SPI_AD_OUT_X_L;
  txbuf[1] = 0xFF;
  txbuf[2] = 0xFF;

  spiSelect(spid);
  spiExchange(spid, 7, txbuf, rxbuf);
  spiUnselect(spid);

  // Swapped for board orientation
  raw[0] = (rxbuf[2] << 8) | rxbuf[1];
  raw[1] = (rxbuf[4] << 8) | rxbuf[3];
  raw[2] = (rxbuf[6] << 8) | rxbuf[5];

  gyroscope_reading_t reading;

  for(int i = 0; i < 3; i++) {
    reading.axes[i] = (float) raw[i] * L3GD20_SENSITIVITY_2000DPS * L3GD20_DPS_TO_RADS;
  }

  return reading;
}

uint8_t L3GD20::readRegister(uint8_t reg) {
  uint8_t txbuf[8];
  uint8_t rxbuf[8];

  txbuf[0] = L3GD20_SPI_RW | reg;
  txbuf[1] = 0xFF;

  spiSelect(this->spid);
  spiExchange(this->spid, 2, txbuf, rxbuf);
  spiUnselect(this->spid);

  return rxbuf[1];
}

void L3GD20::writeRegister(uint8_t reg, uint8_t val) {
  uint8_t txbuf[8];
  uint8_t rxbuf[8];

  txbuf[0] = reg;
  txbuf[1] = val;

  spiSelect(this->spid);
  spiExchange(this->spid, 2, txbuf, rxbuf);
  spiUnselect(this->spid);
}
