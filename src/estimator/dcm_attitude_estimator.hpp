#ifndef DCM_ATTITUDE_ESTIMATOR_HPP_
#define DCM_ATTITUDE_ESTIMATOR_HPP_

#include "Eigen/Dense"

#include "communication/communicator.hpp"
#include "communication/rate_limited_stream.hpp"
#include "estimator/attitude_estimator.hpp"

class DCMAttitudeEstimator : public AttitudeEstimator {
public:
  DCMAttitudeEstimator(Communicator& communicator);

  /**
   * Update the internal DCM and return a new attitude estimate.
   */
  AttitudeEstimate update(const SensorReadingGroup& readings) override;

private:
  /**
   * Return the component vectors to orthogonality. This is accomplished by
   * first bringing the i and j vectors to orthogonality, then drawing the k
   * vector from them.
   */
  void orthonormalize();

  /**
   * Determine the accelerometer correction weight based on the reading. The
   * weighting should have an inverse relationship with the reading's difference
   * from 1g.
   */
  float getAccelWeight(Eigen::Vector3f accel) const;

  AttitudeEstimate makeEstimate(const SensorReadingGroup& readings);

  /**
   * Publish a new message to the output stream if necessary.
   */
  void updateStream();

  Eigen::Matrix3f dcm;
  RateLimitedStream attitudeMessageStream;
};

#endif
