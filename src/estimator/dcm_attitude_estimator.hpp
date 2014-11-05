#ifndef DCM_ATTITUDE_ESTIMATOR_HPP_
#define DCM_ATTITUDE_ESTIMATOR_HPP_

#include <Eigen/Dense>

#include <estimator/attitude_estimator.hpp>

class DCMAttitudeEstimator : public AttitudeEstimator {
public:
  DCMAttitudeEstimator();

  attitude_estimate_t update(accelerometer_reading_t& accel_reading, gyroscope_reading_t& gyro_reading) override;

private:
  Eigen::Matrix3f dcm;

  void orthonormalize();
};

#endif
