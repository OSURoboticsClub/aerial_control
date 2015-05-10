#include "drivers/h3lis331dl.hpp"

#include "unit_config.hpp"
#include "chprintf.h"

void H3LIS331DL::init() {
  // Configure (DS p. 24-28)
  txbuf[0] = h3lis331dl::CTRL_REG1 | (1<<6);   // Auto-increment address
  txbuf[1] = 0b00111111;   // PM: normal mode, DR: 1000Hz, XYZ: enabled
  txbuf[2] = 0b00000000;   // Defaults
  txbuf[3] = 0b00000000;   // Defaults
  txbuf[4] = 0b11110000;   // BDU: on, BLE: data MSB at lower addr, FS: 400g
  txbuf[5] = 0b00000011;   // Turned on
  exchange(6);
}

AccelerometerReading H3LIS331DL::readAccel() {
  AccelerometerReading reading;

  // Poll status
  // TODO(yoos): Do something useful with this
  txbuf[0] = h3lis331dl::STATUS_REG | (1<<7);
  exchange(2);
  uint8_t status = rxbuf[1];

  // Poll accel
  txbuf[0] = h3lis331dl::OUT_X_L | (1<<7);
  exchange(7);

  reading.axes[0] = ((int16_t) ((rxbuf[1]<<8) | rxbuf[2])) * 400.0 / 32768 + accOffsets[0];
  reading.axes[1] = ((int16_t) ((rxbuf[3]<<8) | rxbuf[4])) * 400.0 / 32768 + accOffsets[1];
  reading.axes[2] = ((int16_t) ((rxbuf[5]<<8) | rxbuf[6])) * 400.0 / 32768 + accOffsets[2];

  BaseSequentialStream* chp = (BaseSequentialStream*)&SD4;
  static int i=0;
  if ((i++)%100 == 0) {
    chprintf(chp, "H3: %6d %6d %6d\r\n", reading.axes[0], reading.axes[1], reading.axes[2]);
  }

  return reading;
}

bool H3LIS331DL::healthy() {
  txbuf[0] = h3lis331dl::WHO_AM_I | (1<<7);
  txbuf[1] = 0x00;
  exchange(2);

  BaseSequentialStream* chp = (BaseSequentialStream*)&SD4;
  static int i=0;
  if ((i++)%100 == 0) {
    chprintf(chp, "H3 whoami: %x\r\n", txbuf[1]);
  }

  if (txbuf[1] != 0x32) {
    return false;
  }

  return true;
}
