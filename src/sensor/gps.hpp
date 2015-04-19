#ifndef GPS_HPP_
#define GPS_HPP_

#include <array>

#include "sensor/sensor.hpp"

struct GPSReading {
  bool valid;
  float lat, lon;
  float utc;
};

class GPS : public Sensor {
public:
  virtual void init() = 0;
  virtual GPSReading readGPS() = 0;

  float dmd2float(float dm, char dir) {
    float deg = (int) (dm / 100);
    float m = dm - deg*100;
    float sign = (dir == 'N' || dir == 'E') ? 1 : -1;

    return sign * (deg + m/60);
  }
};

#endif
