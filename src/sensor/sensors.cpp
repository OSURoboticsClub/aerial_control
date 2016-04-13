#include "sensor/sensors.hpp"

Sensors::Sensors(optional<Accelerometer *> accelerometer,
                 optional<Accelerometer *> accelerometerHighRange,
                 optional<Gyroscope *> gyroscope,
                 optional<Barometer *> barometer,
                 optional<GPS *> gps,
                 optional<Magnetometer *> magnetometer)
  : gyroscope(gyroscope),
    accelerometer(accelerometer),
    accelerometerHighRange(accelerometerHighRange),
    barometer(barometer),
    gps(gps),
    magnetometer(magnetometer) {
}

SensorMeasurements Sensors::readAvailableSensors() {
  SensorMeasurements measurements;
  if(accelerometer) measurements.accel = (*accelerometer)->readAccel();
  if(accelerometerHighRange) measurements.accel = (*accelerometerHighRange)->readAccel();
  if(gyroscope) measurements.gyro = (*gyroscope)->readGyro();
  if(barometer) measurements.bar = (*barometer)->readBar();
  if(gps) measurements.gps = (*gps)->readGPS();
  if(magnetometer) measurements.mag = (*magnetometer)->readMag();

  return measurements;
}

bool Sensors::healthy() {
  bool healthy = true;

  if(accelerometer) healthy &= (*accelerometer)->healthy();
  if(accelerometerHighRange) healthy &= (*accelerometerHighRange)->healthy();
  if(gyroscope) healthy &= (*gyroscope)->healthy();
  if(barometer) healthy &= (*barometer)->healthy();
  if(gps) healthy &= (*gps)->healthy();
  if(magnetometer) healthy &= (*magnetometer)->healthy();

  return healthy;
}
