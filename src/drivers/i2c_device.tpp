template <std::size_t tx_size, std::size_t rx_size>
I2CDevice<tx_size, rx_size>::I2CDevice(I2CDriver *i2cd, const I2CConfig *i2ccfg, i2caddr_t addr)
  : i2cd(i2cd), i2ccfg(i2ccfg), addr(addr) {
  i2cStart(i2cd, i2ccfg);
}

template <std::size_t tx_size, std::size_t rx_size>
void I2CDevice<tx_size, rx_size>::exchange() {
  i2cAcquireBus(i2cd);
  i2cMasterTransmit(i2cd, addr, txbuf.data(), tx_size, rxbuf.data(), rx_size);
  i2cReleaseBus(i2cd);
}
