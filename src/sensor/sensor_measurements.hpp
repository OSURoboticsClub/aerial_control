#ifndef SENSOR_MEASUREMENTS_HPP_
#define SENSOR_MEASUREMENTS_HPP_

#include "sensor/accelerometer.hpp"
#include "sensor/barometer.hpp"
#include "sensor/geiger.hpp"
#include "sensor/gps.hpp"
#include "sensor/gyroscope.hpp"
#include "sensor/magnetometer.hpp"

#include "util/optional.hpp"

struct SensorMeasurements {
  optional<AccelerometerReading> accel;
  optional<AccelerometerReading> accelH;
  optional<BarometerReading> bar;
  optional<GeigerReading> ggr;
  optional<GPSReading> gps;
  optional<GyroscopeReading> gyro;
  optional<MagnetometerReading> mag;
};

#endif // SENSOR_MEASUREMENTS_HPP_
