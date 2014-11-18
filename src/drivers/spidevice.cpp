#include <drivers/spidevice.hpp>

SPIDevice::SPIDevice(SPIDriver *spid, const SPIConfig *spicfg, Mutex *spimtx) : spid(spid), spicfg(spicfg), spimtx(spimtx) {
}

void SPIDevice::_spiExchange(uint16_t bufsize, uint8_t *txbuf, uint8_t *rxbuf) {
  chMtxLock(spimtx);

  spiStart(spid, spicfg);                     // Set up transfer parameters.
  spiAcquireBus(spid);                        // Acquire ownership of the bus.
  spiSelect(spid);                            // Assert slave select.

  spiExchange(spid, bufsize, txbuf, rxbuf);   // Atomic transfer operations.

  spiUnselect(spid);                          // Deassert slave select.
  spiReleaseBus(spid);                        // Release ownership.

  chMtxUnlock();
}
