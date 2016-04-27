#ifndef SENSORS_HPP_
#define SENSORS_HPP_

#include "sensor/accelerometer.hpp"
#include "sensor/barometer.hpp"
#include "sensor/gps.hpp"
#include "sensor/gyroscope.hpp"
#include "sensor/magnetometer.hpp"
#include "sensor/sensor_measurements.hpp"
#include "util/optional.hpp"

/**
 * An aggregation of a number of available sensor sources.
 *
 * Sensors are held in optional types to designate whether or not the given
 * sensor is availble. If a sensor is not available, then null optional types
 * will be returned from the relevant methods.
 */
class Sensors {
public:
  Sensors(optional<Accelerometer *> accelerometer,
          optional<Accelerometer *> accelerometerHighRange,
          optional<Gyroscope *> gyroscope,
          optional<Barometer *> barometer,
          optional<GPS *> gps,
          optional<Magnetometer *> magnetometer);

  SensorMeasurements readAvailableSensors();

  /**
   * Perform a calibration step on each available sensor.
   *
   * @see Sensor#calibrateStep()
   */
  void calibrateStep();

  /**
   * Returns whether or not all available sensors are calibrated.
   *
   * @see Sensor#calibrate()
   */
  bool calibrated();

  /**
   * Returns whether or not all available sensors are healthy.
   *
   * @see Sensor#healthy()
   */
  bool healthy();

private:
  optional<Gyroscope *> gyroscope;
  optional<Accelerometer *> accelerometer;
  optional<Accelerometer *> accelerometerHighRange;
  optional<Barometer *> barometer;
  optional<GPS *> gps;
  optional<Magnetometer *> magnetometer;
};

#endif // SENSORS_HPP_
