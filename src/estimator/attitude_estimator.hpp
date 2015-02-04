#ifndef ATTITUDE_ESTIMATOR_HPP_
#define ATTITUDE_ESTIMATOR_HPP_

#include "sensor/accelerometer.hpp"
#include "sensor/gyroscope.hpp"
#include "sensor/magnetometer.hpp"
#include "util/optional.hpp"

struct attitude_estimate_t {
  float roll;
  float pitch;
  float yaw;

  float roll_vel;
  float pitch_vel;
  float yaw_vel;
};

struct sensor_reading_group_t {
  optional<gyroscope_reading_t> gyro;
  optional<accelerometer_reading_t> accel;
  optional<magnetometer_reading_t> mag;
};

class AttitudeEstimator {
public:
  /**
   * Runs the estimator on the latest sensor readings, producing a new attitude
   * estimate.
   */
  virtual attitude_estimate_t update(const sensor_reading_group_t& readings) = 0;
};

#endif
