#ifndef USART_DEVICE_HPP_
#define USART_DEVICE_HPP_

#include <array>
#include <cstdint>

//#include "ch.hpp"
#include "hal.h"

template <std::size_t tx_size, std::size_t rx_size>
class USARTDevice {
public:
  USARTDevice(SerialDriver *sd);

protected:
  void read(std::size_t count);
  void write(std::size_t count);

  SerialDriver *sd;

  std::array<std::uint8_t, tx_size> txbuf;
  std::array<std::uint8_t, rx_size> rxbuf;
};

#include "drivers/usart_device.tpp"

#endif // USART_DEVICE_HPP_
