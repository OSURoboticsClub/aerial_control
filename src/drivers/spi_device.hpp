#ifndef SPI_DEVICE_HPP_
#define SPI_DEVICE_HPP_

#include <array>
#include <cstdint>

#include "hal.h"

template <std::size_t tx_size, std::size_t rx_size>
class SPIDevice {
public:
  SPIDevice(SPIDriver *spid, const SPIConfig *spicfg);

protected:
  void exchange(std::size_t count);

  SPIDriver *spid;
  const SPIConfig *spicfg;

  std::array<std::uint8_t, tx_size> txbuf;
  std::array<std::uint8_t, rx_size> rxbuf;
};

#include "drivers/spi_device.tpp"

#endif // SPI_DEVICE_HPP_
