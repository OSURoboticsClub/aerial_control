#include "drivers/ublox_neo7.hpp"

#include "chprintf.h"
#include "unit_config.hpp"

void UBloxNEO7::init() {
}

gps_reading_t UBloxNEO7::readGPS() {
  gps_reading_t reading;

  // Get all bytes on buffer
  read(10);   // TODO
  //chprintf((BaseSequentialStream*)&SD4, "%10c", rxbuf.data());

  char dbg_buf[16];
  chsnprintf(dbg_buf, 12, "%12c", rxbuf.data());
  chnWriteTimeout((BaseChannel*)&SD4, (uint8_t*)dbg_buf, 12, MS2ST(20));

  // Run parser
  // TODO

  return reading;
}
