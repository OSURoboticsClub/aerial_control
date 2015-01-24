#ifndef DCM_ATTITUDE_ESTIMATOR_HPP_
#define DCM_ATTITUDE_ESTIMATOR_HPP_

#include "Eigen/Dense"

#include "communication/communicator.hpp"
#include "communication/rate_limited_stream.hpp"
#include "estimator/attitude_estimator.hpp"

class DCMAttitudeEstimator : public AttitudeEstimator {
public:
  DCMAttitudeEstimator(Communicator& communicator);

  attitude_estimate_t update(gyroscope_reading_t& gyroReading, accelerometer_reading_t& accelReading) override;

private:
  void orthonormalize();
  float getAccelWeight(Eigen::Vector3f accel);

  Eigen::Matrix3f dcm;
  RateLimitedStream attitudeMessageStream;
};

#endif
