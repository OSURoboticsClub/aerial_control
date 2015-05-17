#include "estimator/atmospheric_location_estimator.hpp"
#include "util/time.hpp"

#include "ch.hpp"
#include "protocol/messages.hpp"

#include "unit_config.hpp"
#include "chprintf.h"

// TODO: Initial location is not valid. Maybe we should be able to mark the
// estimate as invalid until a GPS fix is found?
AtmosphericLocationEstimator::AtmosphericLocationEstimator(Communicator& communicator, Logger& logger)
  : loc{0.0, 0.0, 0.0},
    locationMessageStream(communicator, 10),
    logger(logger) {
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

  // Calculate altitude only if barometer present
  if(meas.bar) {
    static float lastAlt = 0.0;
    lastAlt = loc.alt;

    // Barometer-based estimate
    static float barAlt = 0.0;   // Barometric altitude
    static float dBarAlt = 0.0;
    if ((*meas.bar).pressure > 12) {
      float newAlt = (pow((1000./(*meas.bar).pressure), 1/5.257) - 1) * ((*meas.bar).temperature + 273.15) / 0.0065;
      barAlt = 0.01*newAlt + 0.99*barAlt;   // Moving average
      dBarAlt = barAlt - newAlt;
    }

    // Accelerometer-based estimate
    static float accVel = 0.0;   // Vertical velocity from acceleration
    static float accAlt = 0.0;   // Altitude integrated from acceleration
    accVel += ((*meas.accel).axes[0]-1.0) * 9.80665 / 1000;   // Velocity in m/s
    accAlt += accVel / 1000;   // Altitude in m

    // Correct accel drift with barometer
    accVel = 0.001*dBarAlt + 0.999*accVel;
    accAlt = 0.001*barAlt  + 0.999*accAlt;

    // Update final estimates
    loc.alt = accAlt;
    loc.dAlt = accVel;

    static int i=0;
    if (i == 0) {
      BaseSequentialStream* chp = (BaseSequentialStream*)&SD4;
      chprintf(chp, "%d: %8f %8f\r\n", chibios_rt::System::getTime(), loc.alt, loc.dAlt);
    }
    i = (i+1) % 10;
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
  protocol::message::location_message_t m {
    .time = ST2MS(chibios_rt::System::getTime()),
    .lat = loc.lat,
    .lon = loc.lon,
    .alt = loc.alt
  };

  if(locationMessageStream.ready()) {
    locationMessageStream.publish(m);
  }

  logger.write(m);
}
