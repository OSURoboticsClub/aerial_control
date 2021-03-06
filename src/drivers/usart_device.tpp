template <std::size_t tx_size, std::size_t rx_size>
USARTDevice<tx_size, rx_size>::USARTDevice(SerialDriver *sd)
  : sd(sd) {
}

template <std::size_t tx_size, std::size_t rx_size>
std::size_t USARTDevice<tx_size, rx_size>::readUntil(std::uint8_t stop) {
  // TODO(kyle): Check bounds on rxbuf
  while(!sdGetWouldBlock(sd)) {
    std::uint8_t b = sdGet(sd);
    rxbuf[rxpos++] = b;

    if(b == stop) {
      std::size_t len = rxpos;

      // TODO: Append a null character to facilitate `strtok`?
      rxbuf[rxpos++] = '\0';

      // Reset rxpos to zero in anticipation of next read
      rxpos = 0;

      return len;
    }
  }

  // Stop byte was not found, so 0 bytes are ready to be processed.
  return 0;
}

template <std::size_t tx_size, std::size_t rx_size>
std::size_t USARTDevice<tx_size, rx_size>::read(std::uint8_t maxBytes) {
  rxpos = 0;
  while (!sdGetWouldBlock(sd)) {
    std::uint8_t b = sdGet(sd);
    rxbuf[rxpos++] = b;

    // Return early if we've reached max number of bytes to read
    if (rxpos == maxBytes) {
      return maxBytes;
    }
  }
  return rxpos;
}

template <std::size_t tx_size, std::size_t rx_size>
void USARTDevice<tx_size, rx_size>::write(std::size_t count) {
  // Stuff txbuf
  // TODO

  // Write
  sdAsynchronousWrite(sd, txbuf.data(), count);
}
