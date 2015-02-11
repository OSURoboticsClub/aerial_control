#include "estimator/atmospheric_location_estimator.hpp"

#include "protocol/messages.hpp"

#include "unit_config.hpp"

AtmosphericLocationEstimator::AtmosphericLocationEstimator(Communicator& communicator)
  : locationMessageStream(communicator, 5) {
}

LocationEstimate AtmosphericLocationEstimator::update(const SensorMeasurements& meas) {
  updateStream();

  return makeEstimate(meas);
}

LocationEstimate AtmosphericLocationEstimator::makeEstimate(const SensorMeasurements& meas) {
  // TODO(yoos)
  LocationEstimate estimate = {
    .lat = 1.23,
    .lon = 4.56,
    .alt = 1.2345
  };

  return estimate;
}

void AtmosphericLocationEstimator::updateStream() {
  if(locationMessageStream.ready()) {
    // TODO(yoos): Implement location message.
    //protocol::message::location_message_t m {
    //  .lat = 0,
    //  .lon = 0,
    //  .alt = 0
    //  }
    //};

    //attitudeMessageStream.publish(m);
  }
}
