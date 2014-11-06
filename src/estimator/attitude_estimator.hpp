#ifndef ATTITUDE_ESTIMATOR_HPP_
#define ATTITUDE_ESTIMATOR_HPP_

#include <sensor/accelerometer.hpp>
#include <sensor/gyroscope.hpp>

struct attitude_estimate_t {
  float roll;
  float pitch;
  float yaw;

  float roll_vel;
  float pitch_vel;
  float yaw_vel;
};

class AttitudeEstimator {
public:
  /**
   * Runs the estimator on the latest accelerometer and gyroscope readings,
   * producing a new attitude estimate.
   */
  virtual attitude_estimate_t update(accelerometer_reading_t& accel_reading, gyroscope_reading_t& gyro_reading) =0;
};

#endif

