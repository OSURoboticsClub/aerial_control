#ifndef ATMOSPHERIC_LOCATION_ESTIMATOR_HPP_
#define ATMOSPHERIC_LOCATION_ESTIMATOR_HPP_

#include "communication/communicator.hpp"
#include "communication/rate_limited_stream.hpp"
#include "estimator/location_estimator.hpp"

class AtmosphericLocationEstimator : public LocationEstimator {
public:
  AtmosphericLocationEstimator(Communicator& communicator);

  LocationEstimate update(const SensorMeasurements& meas) override;

private:
  LocationEstimate makeEstimate(const SensorMeasurements& meas);

  /**
   * Publish a new message to the output stream if necessary.
   */
  void updateStream();

  LocationEstimate loc;
  RateLimitedStream locationMessageStream;
};

#endif
