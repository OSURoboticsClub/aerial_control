#ifndef WORLD_ESTIMATOR_HPP_
#define WORLD_ESTIMATOR_HPP_

#include "communication/communicator.hpp"
#include "communication/rate_limited_stream.hpp"
#include "sensor/gps.hpp"

struct world_estimate_t {
  float globe_loc[3];
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
  virtual world_estimate_t update(gps_reading_t& gpsReading);

private:
  RateLimitedStream worldMessageStream;
};

#endif
