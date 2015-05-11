#include "drivers/h3lis331dl.hpp"

#include "unit_config.hpp"
#include "chprintf.h"

void H3LIS331DL::init() {
  // Configure (DS p. 24-28)
  txbuf[0] = h3lis331dl::CTRL_REG1 | (1<<6);   // Auto-increment write address
  txbuf[1] = 0b00111111;   // PM: normal mode, DR: 1000Hz, XYZ: enabled
  txbuf[2] = 0b00000000;   // Defaults
  txbuf[3] = 0b00000000;   // Defaults
  txbuf[4] = 0b11000000;   // BDU: on, BLE: data MSB at lower addr, FS: 100g
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
  txbuf[0] = h3lis331dl::OUT_X_L | (1<<7) | (1<<6);   // Auto-increment read address
  exchange(7);

  reading.axes[0] = ((int16_t) ((rxbuf[1]<<8) | rxbuf[2]))/16 * 0.049 + accOffsets[0];
  reading.axes[1] = ((int16_t) ((rxbuf[3]<<8) | rxbuf[4]))/16 * 0.049 + accOffsets[1];
  reading.axes[2] = ((int16_t) ((rxbuf[5]<<8) | rxbuf[6]))/16 * 0.049 + accOffsets[2];

  // DEBUG
  //BaseSequentialStream* chp = (BaseSequentialStream*)&SD4;
  //static int i=0;
  //if ((i++)%10 == 0) {
  //  chprintf(chp, "H3: %8f %8f %8f\r\n", reading.axes[0], reading.axes[1], reading.axes[2]);
  //}

  return reading;
}

bool H3LIS331DL::healthy() {
  txbuf[0] = h3lis331dl::WHO_AM_I | (1<<7);
  txbuf[1] = 0x00;
  exchange(2);

  if (rxbuf[1] != 0x32) {
    return false;
  }

  return true;
}
