#ifndef DCM_ATTITUDE_ESTIMATOR_HPP_
#define DCM_ATTITUDE_ESTIMATOR_HPP_

#include "Eigen/Dense"

#include "communication/communicator.hpp"
#include "communication/rate_limited_stream.hpp"
#include "estimator/attitude_estimator.hpp"

class DCMAttitudeEstimator : public AttitudeEstimator {
public:
  DCMAttitudeEstimator(Communicator& communicator);

  attitude_estimate_t update(gyroscope_reading_t& gyro_reading, accelerometer_reading_t& accel_reading) override;

private:
  void orthonormalize();

  Eigen::Matrix3f dcm;
  RateLimitedStream attitudeMessageStream;
};

#endif
