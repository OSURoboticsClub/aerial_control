template <std::size_t tx_size, std::size_t rx_size>
SPIDevice<tx_size, rx_size>::SPIDevice(SPIDriver *spid, const SPIConfig *spicfg)
  : spid(spid), spicfg(spicfg) {
}

template <std::size_t tx_size, std::size_t rx_size>
void SPIDevice<tx_size, rx_size>::exchange(std::size_t count) {
  spiStart(spid, spicfg);                               // Set up transfer parameters.
  spiAcquireBus(spid);                                  // Acquire ownership of the bus.
  spiSelect(spid);                                      // Assert slave select.

  spiExchange(spid, count, txbuf.data(), rxbuf.data()); // Atomic transfer operations.

  spiUnselect(spid);                                    // Deassert slave select.
  spiReleaseBus(spid);                                  // Release ownership.
}
