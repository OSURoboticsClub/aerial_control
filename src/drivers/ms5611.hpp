#ifndef MS5611_HPP_
#define MS5611_HPP_

#include <cstdint>

#include "hal.h"

#include "drivers/spi_device.hpp"
#include "sensor/barometer.hpp"

namespace ms5611 {

const std::uint8_t CMD_RESET      = 0x1E;   // W reset chip
const std::uint8_t CMD_CONVERT_D1 = 0x48;   // W convert pressure (OSR=4096)
const std::uint8_t CMD_CONVERT_D2 = 0x58;   // W convert temperature (OSR=4096)
const std::uint8_t CMD_READ       = 0x00;   // R 24-bit pressure data
const std::uint8_t CMD_PROM_SETUP = 0xA0;   // R 8x 2 bytes factory and calibration data
const std::uint8_t CMD_PROM_C1    = 0xA2;   // R 6x 2 bytes calibration data

}

class MS5611 : protected SPIDevice<8, 8>, public Barometer {
public:
  using SPIDevice::SPIDevice;

  void init() override;
  BarometerReading readBar() override;

private:
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t val);
  bool healthy();

  /**
   * Update temperature-compensated pressure and temperature
   */
  void updatePT(void);

  uint32_t C1, C2, C3, C4, C5, C6;   // These are really 16-bit values, but we'll just make them 32 bits wide so we can bit shift conveniently.
  uint32_t D1, D2;
  float pressure, temperature;
};

#endif
