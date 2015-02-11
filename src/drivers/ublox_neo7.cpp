#include "drivers/ublox_neo7.hpp"

#include "chprintf.h"
#include "unit_config.hpp"

void UBloxNEO7::init() {
}

GPSReading UBloxNEO7::readGPS() {
  GPSReading reading;

  // Read all available bytes until the newline character. NMEA dictates that
  // messages should end with a CRLF, but we'll only look for the LF.
  std::size_t len = readUntil('\n');

  // Check if a full line is ready to be processed
  if(len > 0) {
    // TODO: Run parser
  }

  // TODO
  reading.lat = 1.2f;
  reading.lon = 3.4f;

  //char dbg_buf[16];
  //chsnprintf(dbg_buf, 12, "%12c", rxbuf.data());
  //chnWriteTimeout((BaseChannel*)&SD4, (uint8_t*)dbg_buf, 12, MS2ST(20));

  return reading;
}
