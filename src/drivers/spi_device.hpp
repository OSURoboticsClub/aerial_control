#ifndef SPI_DEVICE_HPP_
#define SPI_DEVICE_HPP_

#include "hal.h"

class SPIDevice {
public:
  explicit SPIDevice(SPIDriver *spid, const SPIConfig *spicfg);

protected:
  SPIDriver *spid;
  const SPIConfig *spicfg;

  // TODO(yoos): Rename
  void _spiExchange(uint16_t bufsize, uint8_t *txbuf, uint8_t *rxbuf);
};

#endif // SPI_DEVICE_HPP_
