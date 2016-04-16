#ifndef SENSORS_HPP_
#define SENSORS_HPP_

#include "sensor/accelerometer.hpp"
#include "sensor/barometer.hpp"
#include "sensor/gps.hpp"
#include "sensor/gyroscope.hpp"
#include "sensor/magnetometer.hpp"
#include "sensor/sensor_measurements.hpp"
#include "util/optional.hpp"

class Sensors {
public:
  Sensors(optional<Accelerometer *> accelerometer,
          optional<Accelerometer *> accelerometerHighRange,
          optional<Gyroscope *> gyroscope,
          optional<Barometer *> barometer,
          optional<GPS *> gps,
          optional<Magnetometer *> magnetometer);

  SensorMeasurements readAvailableSensors();
  void calibrateStep();

  bool healthy();
  bool calibrated();

private:
  optional<Gyroscope *> gyroscope;
  optional<Accelerometer *> accelerometer;
  optional<Accelerometer *> accelerometerHighRange;
  optional<Barometer *> barometer;
  optional<GPS *> gps;
  optional<Magnetometer *> magnetometer;
};

#endif // SENSORS_HPP_
