#include <hal.h>

#include <spi_config.hpp>

void spiPlatformInit(void) {
  //spiInit();
  palSetPadMode(GPIOB, 3, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);   /* SPI1 SCK */
  palSetPadMode(GPIOB, 4, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);   /* SPI1 MISO */
  palSetPadMode(GPIOB, 5, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);   /* SPI1 MOSI */
  palSetPadMode(GPIOB, 2, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);   /* MPU-6000 CS */
  palSetPad(GPIOB, 2);   /* Unselect */

  chMtxInit(&spi_mtx);
}

void _spiExchange(SPIDriver *spid, const SPIConfig *spicfg, uint16_t bufsize, uint8_t *txbuf, uint8_t *rxbuf) {
  chMtxLock(&spi_mtx);

  spiStart(spid, spicfg);                     // Set up transfer parameters.
  spiAcquireBus(spid);                        // Acquire ownership of the bus.
  spiSelect(spid);                            // Assert slave select.

  spiExchange(spid, bufsize, txbuf, rxbuf);   // Atomic transfer operations.

  spiUnselect(spid);                          // Deassert slave select.
  spiReleaseBus(spid);                        // Release ownership.

  chMtxUnlock();
}
