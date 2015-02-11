#include "estimator/world_estimator.hpp"

#include <cstdio>
#include "hal.h"

#include "unit_config.hpp"

WorldEstimator::WorldEstimator(
    LocationEstimator& locEstimator,
    AttitudeEstimator& attEstimator,
    Communicator& communicator)
  : locEstimator(locEstimator),
    attEstimator(attEstimator),
    worldMessageStream(communicator, 10) {
}

WorldEstimate WorldEstimator::update(const SensorMeasurements& meas) {
  // TODO: get GPS data

  WorldEstimate estimate {
    .loc = locEstimator.update(meas),
    .att = attEstimator.update(meas)
  };

  if(worldMessageStream.ready()) {
    protocol::message::log_message_t m;
    sprintf(m.data, "therm: %f", meas.therm->celsius);

    worldMessageStream.publish(m);
  }

  return estimate;
}
