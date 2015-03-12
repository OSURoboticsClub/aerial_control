#ifndef GPS_HPP_
#define GPS_HPP_

#include <array>

enum class Direction {
  NORTH,
  EAST,
  SOUTH,
  WEST,
  INVALID
};

struct GPSReading {
  bool valid;
  float lat;
  Direction latDir;
  float lon;
  Direction lonDir;
};

class GPS {
public:
  virtual void init() = 0;
  virtual GPSReading readGPS() = 0;
};

#endif
