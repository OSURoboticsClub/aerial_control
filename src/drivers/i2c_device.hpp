#ifndef I2C_DEVICE_HPP_
#define I2C_DEVICE_HPP_

#include <array>
#include <cstdint>

#include "hal.h"

template <std::size_t tx_size, std::size_t rx_size>
class I2CDevice {
public:
  I2CDevice(I2CDriver *i2cd, const I2CConfig *i2ccfg, const i2caddr_t addr);

protected:
  void exchange(std::size_t tx, std::size_t rx);

  I2CDriver *i2cd;
  const I2CConfig *i2ccfg;
  const i2caddr_t addr;

  std::array<std::uint8_t, tx_size> txbuf;
  std::array<std::uint8_t, rx_size> rxbuf;
};

#include "drivers/i2c_device.tpp"

#endif // I2C_DEVICE_HPP_
