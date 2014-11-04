#ifndef ATTITUDE_ESTIMATOR_HPP_
#define ATTITUDE_ESTIMATOR_HPP_

#include <sensor/accelerometer.hpp>
#include <sensor/gyroscope.hpp>

struct attitude_estimate_t {
  float pitch;
  float roll;
  float yaw;

  float pitch_vel;
  float roll_vel;
  float yaw_vel;
};

class AttitudeEstimator {
public:
  virtual attitude_estimate_t update(accelerometer_reading_t& accel_reading, gyroscope_reading_t& gyro_reading) =0;
};

#endif

