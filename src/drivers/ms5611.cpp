#include "drivers/ms5611.hpp"

#include "unit_config.hpp"

#include "chprintf.h"

void MS5611::init() {
  // Reset device.
  txbuf[0] = ms5611::CMD_RESET;
  exchange(1);

  // Wait for 2.8ms reload (DS p. 9)
  chThdSleepMicroseconds(3000);

  // Read factory data and setup
  txbuf[0] = ms5611::CMD_PROM_SETUP | (1<<7);
  exchange(3);
  // TODO(yoos): health check

  // Read calibration data
  txbuf[0] = ms5611::CMD_PROM_C1;
  exchange(3);
  C1 = (rxbuf[1]<<8) | rxbuf[2];

  txbuf[0] = ms5611::CMD_PROM_C1+2;
  exchange(3);
  C2 = (rxbuf[1]<<8) | rxbuf[2];

  txbuf[0] = ms5611::CMD_PROM_C1+4;
  exchange(3);
  C3 = (rxbuf[1]<<8) | rxbuf[2];

  txbuf[0] = ms5611::CMD_PROM_C1+6;
  exchange(3);
  C4 = (rxbuf[1]<<8) | rxbuf[2];

  txbuf[0] = ms5611::CMD_PROM_C1+8;
  exchange(3);
  C5 = (rxbuf[1]<<8) | rxbuf[2];

  txbuf[0] = ms5611::CMD_PROM_C1+10;
  exchange(3);
  C6 = (rxbuf[1]<<8) | rxbuf[2];
}

BarometerReading MS5611::readBar() {
  // Read every 10 loops to leave time for the maximum 9.04 ms conversion time.
  // This assumes DT = 1 ms. We flip-flop between reading pressure and
  // temperature, sending the conversion command for the other measurement type
  // at the end of each loop.
  static int loop = 0;
  static int which = 0;
  if (loop == 0) {
    txbuf[0] = ms5611::CMD_READ;
    exchange(4);
    uint32_t raw = (((uint32_t) rxbuf[1])<<16) | (((uint32_t) rxbuf[2])<<8) | rxbuf[3];

    if (which == 0) {
      D1 = raw;   // Pressure
      txbuf[0] = ms5611::CMD_CONVERT_D2;   // Convert temperature
      exchange(1);
    }
    else {
      D2 = raw;   // Temperature
      txbuf[0] = ms5611::CMD_CONVERT_D1;   // Convert pressure
      exchange(1);

      updatePT();   // Update temperature-compensated pressure
    }
    which = 1-which;
  }
  loop = (loop+1) % 10;

  // Pack data
  BarometerReading reading {
    .pressure = pressure,
    .temperature = temperature
  };

  return reading;
}

void MS5611::updatePT(void) {
  // Calculate temperature (-40 to 85 deg C with 0.01 deg resolution)
  int32_t dT = D2 - (C5 << 8);   // Actual temp - Reference temp
  int32_t TEMP = 2000 + ((dT * C6) >> 23);   // 100x actual temperature

  // Calculate temperature compensation offsets
  int64_t OFF  = (C2 << 16) + ((C4 * dT) >> 7);   // Offset at actual temp
  int64_t SENS = (C1 << 15) + ((C3 * dT) >> 8);   // Sensitivity at actual temp

  // Second-order temperature compensation
  int32_t T2 = 0;
  int64_t OFF2 = 0;
  int64_t SENS2 = 0;
  if (TEMP < 2000) {
    T2 = (dT * dT) >> 31;
    OFF2 = (5 * (TEMP - 2000) * (TEMP - 2000)) >> 1;
    SENS2 = OFF2 >> 1;
  }
  if (TEMP < -1500) {
    OFF2 += 7 * (TEMP + 1500) * (TEMP + 1500);
    SENS2 += (11 * (TEMP + 1500) * (TEMP + 1500)) >> 1;
  }
  TEMP -= T2;
  OFF  -= OFF2;
  SENS -= SENS2;

  // Calculate pressure (10 to 1200 mbar with 0.01 mbar resolution)
  int32_t P = (((D1 * SENS) >> 21) - OFF) >> 15;   // 100x temp-compensated pressure
  //chprintf((BaseSequentialStream*)&SD4, "PT: %d %d\r\n", P, TEMP);

  pressure = P / 100.;
  temperature = TEMP / 100.;
}

bool MS5611::healthy() {
  return true;
}
