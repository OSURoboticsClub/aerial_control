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
  /**
   * While data is available in the serial buffer, continue appending to the
   * internal buffer until the `stop` byte is encountered. Does not block.
   *
   * Returns the number of bytes available if the the `stop` byte was found.
   * Otherwise, return 0.
   */
  std::size_t readUntil(std::uint8_t stop);

  /**
   * Read up to maxBytes bytes. If maxBytes is 0, read all available bytes. Does not
   * block.
   *
   * Returns the number of bytes available.
   */
  std::size_t read(std::uint8_t maxBytes);

  void write(std::size_t count);

  SerialDriver *sd;

  std::array<std::uint8_t, tx_size> txbuf;

  std::size_t rxpos;
  std::array<std::uint8_t, rx_size> rxbuf;
};

#include "drivers/usart_device.tpp"

#endif // USART_DEVICE_HPP_
