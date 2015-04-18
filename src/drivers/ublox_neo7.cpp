#include "drivers/ublox_neo7.hpp"

#include <cstdlib>
#include <cstring>

#include "chprintf.h"
#include "unit_config.hpp"

const char NMEA_LF = '\n';
const char *NMEA_DELIMS = ",\0";

struct GPGLLMessage {
  float lon;
  char lonDir;
  float lat;
  char latDir;
  float utc;
  char valid;
};

void UBloxNEO7::init() {
}

GPSReading UBloxNEO7::readGPS() {
  // Read all available bytes until the newline character. NMEA dictates that
  // messages should end with a CRLF, but we'll only look for the LF.
  std::size_t len = readUntil(NMEA_LF);

  //static int loop = 0;
  //if (loop % 100 == 0) {
  //  chprintf((BaseSequentialStream*)sd, "$PUBX,00*33\r\n");
  //}
  //loop = (loop+1) % 100;

  // Check if a full line is ready to be processed
  if(len > 0) {
    char *start = reinterpret_cast<char *>(rxbuf.data());
    //chprintf((BaseSequentialStream*)&SD4, "%s", rxbuf.data());

    // Skip over the leading "$"
    start += 1;

    char *token = std::strtok(start, NMEA_DELIMS);

    // Only care about GPGLL messages
    if(std::strcmp("GPGLL", token) == 0) {
      GPGLLMessage message;
      int position = 0;

      while((token = std::strtok(nullptr, NMEA_DELIMS)) != nullptr) {
        switch(position++) {
          case 0:
            message.lat = atof(token);
            break;
          case 1:
            message.latDir = token[0];
            break;
          case 2:
            message.lon = atof(token);
            break;
          case 3:
            message.lonDir = token[0];
            break;
          case 4:
            message.utc = atof(token);
            break;
          case 5:
            message.valid = token[0];
            break;
        };
      }

      return GPSReading {
        // Make sure we got all parts of the message. If fields are omitted in
        // the message then `strtok` will skip over repeated delimiters and the
        // above loop will complete before all delimiters were found.
        .valid = message.valid && position == 7,
        .lat = dmd2float(message.lat, message.latDir),
        .lon = dmd2float(message.lon, message.lonDir),
        .utc = message.utc
      };
    }
  }

  // TODO: Return previous message with old timestamp.
  return GPSReading {
    .valid = false,
      .lat = 10000.0,
      .lon = 10000.0,
      .utc = 0.0
  };
}

bool UBloxNEO7::healthy() {
  return true;
}
