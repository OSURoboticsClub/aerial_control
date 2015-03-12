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
  // TODO(yoos): Turn off default (1Hz?!) sentences and poll at 10Hz.
  //chprintf((BaseSequentialStream*)&SD6, "$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n");
  //chprintf((BaseSequentialStream*)&SD6, "$PUBX,40,GGA,0,0,0,0,0,0*5A\r\n");
  //chprintf((BaseSequentialStream*)&SD6, "$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n");
  //chprintf((BaseSequentialStream*)&SD6, "$PUBX,40,RMC,0,0,0,0,0,0*47\r\n");
  //chprintf((BaseSequentialStream*)&SD6, "$PUBX,40,GSV,0,0,0,0,0,0*59\r\n");
  //chprintf((BaseSequentialStream*)&SD6, "$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n");
}

GPSReading UBloxNEO7::readGPS() {
  // Read all available bytes until the newline character. NMEA dictates that
  // messages should end with a CRLF, but we'll only look for the LF.
  std::size_t len = readUntil(NMEA_LF);

  //static int loop = 0;
  //if (loop % 100 == 0) {
  //  chprintf((BaseSequentialStream*)&SD6, "$PUBX,00*33\r\n");
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

      while(token != nullptr) {
        token = std::strtok(nullptr, NMEA_DELIMS);

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
        .valid = true,
        .lat = dmd2float(message.lat, message.latDir),
        .lon = dmd2float(message.lon, message.lonDir),
        .utc = message.utc
      };
    }
  } else {
    // TODO: Return previous message with old timestamp.
      return GPSReading {
        .valid = false,
        .lat = 0.0,
        .lon = 0.0,
        .utc = 0.0
      };
  }
}
