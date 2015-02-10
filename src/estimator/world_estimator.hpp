#ifndef WORLD_ESTIMATOR_HPP_
#define WORLD_ESTIMATOR_HPP_

#include "communication/communicator.hpp"
#include "communication/rate_limited_stream.hpp"
#include "estimator/attitude_estimator.hpp"

struct WorldEstimate {
  float globe_loc[3];
  float therm;
};

class WorldEstimator {
public:
  WorldEstimator(Communicator& communicator);

  /**
   * Runs the estimator on the latest GPS readings, producing a new world
   * estimate.
   *
   * TODO(yoos): Consolidate with attitude estimator.
   */
  virtual WorldEstimate update(const SensorReadingGroup& readings);

private:
  RateLimitedStream worldMessageStream;
};

#endif
