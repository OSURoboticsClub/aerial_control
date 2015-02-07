template <std::size_t tx_size, std::size_t rx_size>
USARTDevice<tx_size, rx_size>::USARTDevice(SerialDriver *sd)
  : sd(sd) {
}

template <std::size_t tx_size, std::size_t rx_size>
void USARTDevice<tx_size, rx_size>::read(std::size_t count) {
  // Read
  sdAsynchronousRead(sd, rxbuf, 10);

  // Do something with it
}

template <std::size_t tx_size, std::size_t rx_size>
void USARTDevice<tx_size, rx_size>::write(std::size_t count) {
  // Stuff txbuf
  // TODO

  // Write
  sdAsynchronousWrite(sd, txbuf, 10);
}
