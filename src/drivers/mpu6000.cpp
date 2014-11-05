#include <drivers/mpu6000.hpp>

#include <hal.h>

#include <hal_config.hpp>

MPU6000::MPU6000(SPIDriver *spi) : spi(spi) {
}

void MPU6000::init() {
  // Wake up device, enable X, Y, and Z outputs, and set 760Hz mode.
  //writeRegister(mpu6000_SPI_AD_CTRL_REG1, 0x0F | (1 << 7) | (1 << 6));

  // Set 2000 DPS mode
  //writeRegister(mpu6000_SPI_AD_CTRL_REG4, (1 << 5) | (1 << 4));
}

gyroscope_reading_t MPU6000::readGyro() {
  gyroscope_reading_t reading;

  // TODO: Implement

  return reading;
}

accelerometer_reading_t MPU6000::readAccel() {
  uint8_t txbuf[8];
  uint8_t rxbuf[8];
  int16_t raw_gyr[3];
  int16_t raw_acc[3];

  txbuf[0] = 0;//mpu6000_SPI_RW | mpu6000_SPI_MS | mpu6000_SPI_AD_OUT_X_L;
  txbuf[1] = 0xFF;
  txbuf[2] = 0xFF;

  spiSelect(spi);
  //spiExchange(spi, 7, txbuf, rxbuf);
  spiUnselect(spi);

  // Swapped for board orientation
  raw_gyr[0] = (rxbuf[4] << 8) | rxbuf[3];
  raw_gyr[1] = (rxbuf[2] << 8) | rxbuf[1];
  raw_gyr[2] = (rxbuf[6] << 8) | rxbuf[5];

  accelerometer_reading_t reading;

  // for(int i = 0; i < 3; i++) {
  //   reading.gyr[i] = 0;//(float) raw[i] * mpu6000_SENSITIVITY_2000DPS * mpu6000_DPS_TO_RADS;
  // }

  return reading;
}

uint8_t MPU6000::readRegister(uint8_t reg) {
  uint8_t txbuf[8];
  uint8_t rxbuf[8];

  txbuf[0] = 0;//mpu6000_SPI_RW | reg;
  txbuf[1] = 0xFF;

  spiSelect(this->spi);
  //spiExchange(this->spi, 2, txbuf, rxbuf);
  spiUnselect(this->spi);

  return rxbuf[1];
}

void MPU6000::writeRegister(uint8_t reg, uint8_t val) {
  uint8_t txbuf[8];
  uint8_t rxbuf[8];

  txbuf[0] = reg;
  txbuf[1] = val;

  spiSelect(this->spi);
  //spiExchange(this->spi, 2, txbuf, rxbuf);
  spiUnselect(this->spi);
}
