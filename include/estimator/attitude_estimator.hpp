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
  virtual struct attitude_estimate_t update(struct accelerometer_reading_t& accel_reading, struct gyroscope_reading_t& gyro_reading) =0;
};

#endif

