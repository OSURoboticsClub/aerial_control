#include <drivers/mpu6000.hpp>

#include <hal.h>

#include <hal_config.hpp>

MPU6000::MPU6000(SPIDriver *spid, const SPIConfig *spicfg) : spid(spid), spicfg(spicfg) {
}

void MPU6000::init() {
  uint8_t txbuf[8], rxbuf[8];

  // Start bus
  chMtxLock(&spi_mtx);
  spiAcquireBus(spid);
  spiStart(spid, spicfg);
  spiSelect(spid);

  // Reset device.
  txbuf[0] = MPU6000_PWR_MGMT_1;
  txbuf[1] = 0x80;
  spiExchange(spid, 2, txbuf, rxbuf);   // Exchange data.

  chThdSleepMilliseconds(100);

  // Set sample rate to 1 kHz.
  txbuf[0] = MPU6000_SMPLRT_DIV;
  txbuf[1] = 0x00;
  spiExchange(spid, 2, txbuf, rxbuf);   // Exchange data.

  // Set DLPF to 4 (20 Hz gyro bandwidth1 Hz accelerometer bandwidth)
  txbuf[0] = MPU6000_CONFIG;
  txbuf[1] = 0x04;
  spiExchange(spid, 2, txbuf, rxbuf);   // Exchange data.

  // Wake up device and set clock source to Gyro Z.
  txbuf[0] = MPU6000_PWR_MGMT_1;
  txbuf[1] = 0x03;
  spiExchange(spid, 2, txbuf, rxbuf);   // Exchange data.

  // Set gyro full range to 2000 dps. We first read in the current register
  // value so we can change what we need and leave everything else alone.
  txbuf[0] = MPU6000_GYRO_CONFIG | (1<<7);
  spiExchange(spid, 2, txbuf, rxbuf);   // Exchange data.
  txbuf[0] = MPU6000_GYRO_CONFIG;
  txbuf[1] = (rxbuf[1] & ~0x18) | 0x18;
  spiExchange(spid, 2, txbuf, rxbuf);   // Exchange data.

  // Set accelerometer full range to 2 g.
  txbuf[0] = MPU6000_ACCEL_CONFIG | (1<<7);
  spiExchange(spid, 2, txbuf, rxbuf);   // Exchange data.
  txbuf[0] = MPU6000_ACCEL_CONFIG;
  txbuf[1] = (rxbuf[1] & ~0x18) | 0x00;
  spiExchange(spid, 2, txbuf, rxbuf);   // Exchange data.

  // Stop bus
  spiUnselect(spid);
  spiReleaseBus(spid);
  chMtxUnlock();

  // Read once to clear out bad data?
  readGyro();
  readAccel();
}

gyroscope_reading_t MPU6000::readGyro() {
  uint8_t txbuf[8], rxbuf[8];
  gyroscope_reading_t reading;

  // Start bus
  chMtxLock(&spi_mtx);
  spiAcquireBus(spid);
  spiStart(spid, spicfg);
  spiSelect(spid);

  // Get data
  txbuf[0] = MPU6000_GYRO_XOUT_H | (1<<7);
  spiExchange(spid, 7, txbuf, rxbuf);
  reading.axes[0] = ((int16_t) ((rxbuf[1]<<8) | rxbuf[2])) / 16.384 * 3.1415926535 / 180.0 + GYR_X_OFFSET;
  reading.axes[1] = ((int16_t) ((rxbuf[3]<<8) | rxbuf[4])) / 16.384 * 3.1415926535 / 180.0 + GYR_Y_OFFSET;
  reading.axes[2] = ((int16_t) ((rxbuf[5]<<8) | rxbuf[6])) / 16.384 * 3.1415926535 / 180.0 + GYR_Z_OFFSET;

  // TODO(yoos): correct for thermal bias.
  txbuf[0] = MPU6000_TEMP_OUT_H | (1<<7);
  spiExchange(spid, 3, txbuf, rxbuf);
  float temp = ((int16_t) ((rxbuf[1]<<8) | rxbuf[2])) / 340 + 36.53;

  // Stop bus
  spiUnselect(spid);
  spiReleaseBus(spid);
  chMtxUnlock();

  return reading;
}

accelerometer_reading_t MPU6000::readAccel() {
  uint8_t txbuf[8], rxbuf[8];
  accelerometer_reading_t reading;

  // Start bus
  chMtxLock(&spi_mtx);
  spiAcquireBus(spid);
  spiStart(spid, spicfg);
  spiSelect(spid);

  // Get data
  txbuf[0] = MPU6000_ACCEL_XOUT_H | (1<<7);
  spiExchange(spid, 7, txbuf, rxbuf);
  reading.axes[0] = ((int16_t) ((rxbuf[1]<<8) | rxbuf[2])) / 16384.0 + ACC_X_OFFSET;
  reading.axes[1] = ((int16_t) ((rxbuf[3]<<8) | rxbuf[4])) / 16384.0 + ACC_Y_OFFSET;
  reading.axes[2] = ((int16_t) ((rxbuf[5]<<8) | rxbuf[6])) / 16384.0 + ACC_Z_OFFSET;

  // Stop bus
  spiUnselect(spid);
  spiReleaseBus(spid);
  chMtxUnlock();

  return reading;
}

//uint8_t MPU6000::readRegister(uint8_t reg) {
//  uint8_t txbuf[8];
//  uint8_t rxbuf[8];
//
//  txbuf[0] = reg;
//  txbuf[1] = 0xFF;
//
//  spiSelect(this->spid);
//  spiExchange(this->spid, 2, txbuf, rxbuf);
//  spiUnselect(this->spid);
//
//  return rxbuf[1];
//}
//
//void MPU6000::writeRegister(uint8_t reg, uint8_t val) {
//  uint8_t txbuf[8];
//  uint8_t rxbuf[8];
//
//  txbuf[0] = reg;
//  txbuf[1] = val;
//
//  spiSelect(this->spid);
//  spiExchange(this->spid, 2, txbuf, rxbuf);
//  spiUnselect(this->spid);
//}
