#include "drivers/ublox_neo7.hpp"

#include "unit_config.hpp"

void UBloxNEO7::init() {
}

gps_reading_t UBloxNEO7::readGPS() {
  gps_reading_t reading;

  // Get all bytes on buffer

  // Just print for now

  //char dbg_buf[16];
  //for (int i=0; i<16; i++) {
  //  dbg_buf[i] = 0;
  //}
  //chsnprintf(dbg_buf, 12, "%0.6f\r\n", reading.axes[2]);
  //chnWriteTimeout((BaseChannel*)&SD3, (uint8_t*)dbg_buf, 12, MS2ST(20));

  return reading;
}
