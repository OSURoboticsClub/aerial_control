#ifndef DCM_ATTITUDE_ESTIMATOR_HPP_
#define DCM_ATTITUDE_ESTIMATOR_HPP_

#include <Eigen/Dense>

#include <estimator/attitude_estimator.hpp>

class DCMAttitudeEstimator : public AttitudeEstimator {
public:
  DCMAttitudeEstimator();

  attitude_estimate_t update(gyroscope_reading_t& gyro_reading, accelerometer_reading_t& accel_reading) override;

private:
  Eigen::Matrix3f dcm;

  void orthonormalize();
};

#endif
