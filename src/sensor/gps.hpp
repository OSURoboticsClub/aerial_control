#ifndef GPS_HPP_
#define GPS_HPP_

#include <array>

struct gps_reading_t {
  float lat, lon;
};

class GPS {
public:
  virtual void init() = 0;
  virtual gps_reading_t readGPS() = 0;
};

#endif
