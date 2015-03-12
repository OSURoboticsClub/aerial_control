#include "estimator/atmospheric_location_estimator.hpp"

#include "protocol/messages.hpp"

#include "unit_config.hpp"

AtmosphericLocationEstimator::AtmosphericLocationEstimator(Communicator& communicator)
  : locationMessageStream(communicator, 5) {
}

LocationEstimate AtmosphericLocationEstimator::update(const SensorMeasurements& meas) {
  makeEstimate(meas);
  updateStream();

  return loc;
}

LocationEstimate AtmosphericLocationEstimator::makeEstimate(const SensorMeasurements& meas) {
  // TODO(yoos)
  loc.lat = 1.23;
  loc.lon = 4.56;
  loc.alt = (*meas.bar).pressure;

  return loc;
}

void AtmosphericLocationEstimator::updateStream() {
  if(locationMessageStream.ready()) {
    // TODO(yoos): Implement location message.
    protocol::message::location_message_t m {
      .lat = loc.lat,
      .lon = loc.lon,
      .alt = loc.alt
    };

    locationMessageStream.publish(m);
  }
}
