#ifndef SENSOR_MEASUREMENTS_HPP_
#define SENSOR_MEASUREMENTS_HPP_

#include "sensors/accelerometer.hpp"
#include "sensors/gps.hpp"
#include "sensors/gyroscope.hpp"
#include "sensors/magnetometer.hpp"

struct SensorMeasurements {
  optional<AccelerometerReading> accel;
  optional<GyroscopeReading> gyro;
  optional<GPSReading> gps;
  optional<MagnetometerReading> mag;
};

#endif
