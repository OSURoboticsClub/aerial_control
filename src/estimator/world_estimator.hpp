#ifndef WORLD_ESTIMATOR_HPP_
#define WORLD_ESTIMATOR_HPP_

#include "communication/communicator.hpp"
#include "communication/rate_limited_stream.hpp"
#include "estimator/location_estimator.hpp"
#include "estimator/attitude_estimator.hpp"

#include "util/optional.hpp"

struct WorldEstimate {
  LocationEstimate loc;
  AttitudeEstimate att;
};

class WorldEstimator {
public:
  WorldEstimator(LocationEstimator& locEstimator, AttitudeEstimator& attEstimator, Communicator& communicator);

  LocationEstimator& locEstimator;
  AttitudeEstimator& attEstimator;

  /**
   * Run location and attitude estimators on the latest sensor measurements,
   * producing a new world estimate.
   */
  WorldEstimate update(const SensorMeasurements& meas);

private:
  RateLimitedStream worldMessageStream;
};

#endif
