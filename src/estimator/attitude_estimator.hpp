#ifndef ATTITUDE_ESTIMATOR_HPP_
#define ATTITUDE_ESTIMATOR_HPP_

#include "sensor/accelerometer.hpp"
#include "sensor/gyroscope.hpp"
#include "sensor/magnetometer.hpp"
#include "util/optional.hpp"

struct AttitudeEstimate {
  float roll;
  float pitch;
  float yaw;

  float rollVel;
  float pitchVel;
  float yawVel;

  float rollAcc;
  float pitchAcc;
  float yawAcc;
};

struct SensorReadingGroup {
  optional<GyroscopeReading> gyro;
  optional<AccelerometerReading> accel;
  optional<MagnetometerReading> mag;
};

class AttitudeEstimator {
public:
  /**
   * Runs the estimator on the latest sensor readings, producing a new attitude
   * estimate.
   */
  virtual AttitudeEstimate update(const SensorReadingGroup& readings) = 0;
};

#endif
