#include <drivers/spi_device.hpp>

SPIDevice::SPIDevice(SPIDriver *spid, const SPIConfig *spicfg) : spid(spid), spicfg(spicfg) {
}

void SPIDevice::_spiExchange(uint16_t bufsize, uint8_t *txbuf, uint8_t *rxbuf) {
  spiStart(spid, spicfg);                     // Set up transfer parameters.
  spiAcquireBus(spid);                        // Acquire ownership of the bus.
  spiSelect(spid);                            // Assert slave select.

  spiExchange(spid, bufsize, txbuf, rxbuf);   // Atomic transfer operations.

  spiUnselect(spid);                          // Deassert slave select.
  spiReleaseBus(spid);                        // Release ownership.
}
