#ifndef GPS_HPP_
#define GPS_HPP_

#include <array>

struct GPSReading {
  float lat, lon;
};

class GPS {
public:
  virtual void init() = 0;
  virtual GPSReading readGPS() = 0;
};

#endif
