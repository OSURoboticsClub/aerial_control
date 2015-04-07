#include "estimator/atmospheric_location_estimator.hpp"

#include "ch.hpp"
#include "protocol/messages.hpp"

#include "unit_config.hpp"

// TODO: Initial location is not valid. Maybe we should be able to mark the
// estimate as invalid until a GPS fix is found?
AtmosphericLocationEstimator::AtmosphericLocationEstimator(Communicator& communicator)
  : loc{0.0, 0.0, 0.0},
    locationMessageStream(communicator, 50) {
}

LocationEstimate AtmosphericLocationEstimator::update(const SensorMeasurements& meas) {
  makeEstimate(meas);
  updateStream();

  return loc;
}

LocationEstimate AtmosphericLocationEstimator::makeEstimate(const SensorMeasurements& meas) {
  // TODO: Ideally, we could integrate perfectly continuous IMU data to get
  // a perfect location fix. Since we have to deal with discrete timesteps,
  // figure out how to throw GPS and barometer readings into the mix for error
  // correction.
  //
  // For now, we use the GPS and barometer readings only.
  if(meas.gps && (*meas.gps).valid) {
    loc.lat = (*meas.gps).lat;
    loc.lon = (*meas.gps).lon;
  }

  // Altitude
  static float lastAlt = 0.0;
  if(meas.bar) {
    lastAlt = loc.alt;
    if ((*meas.bar).pressure > 12) {
      float newAlt = (pow((1000./(*meas.bar).pressure), 1/5.257) - 1) * ((*meas.bar).temperature + 273.15) / 0.0065;
      loc.alt = 0.02*newAlt + 0.98*loc.alt;   // Moving average
    }

    // Rate of change
    loc.dAlt = (loc.alt - lastAlt) * 20.;   // 50 Hz altimetry
  }

  // Jerk
  static float lastAcc[3];
  for (int i=0; i<3; i++) {
    loc.jerk[i] = ((*meas.accel).axes[i]) - lastAcc[i];
    lastAcc[i] = (*meas.accel).axes[i];
  }

  return loc;
}

void AtmosphericLocationEstimator::updateStream() {
  if(locationMessageStream.ready()) {
    protocol::message::location_message_t m {
      .time = ST2MS(chibios_rt::System::getTime()),
      .lat = loc.lat,
      .lon = loc.lon,
      .alt = loc.alt
    };

    locationMessageStream.publish(m);
  }
}
