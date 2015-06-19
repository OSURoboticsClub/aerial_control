#include "estimator/world_estimator.hpp"

#include <cstdio>
#include "hal.h"

#include "unit_config.hpp"
#include "util/time.hpp"
#include <chprintf.h>

WorldEstimator::WorldEstimator(
    LocationEstimator& locEstimator,
    AttitudeEstimator& attEstimator,
    Communicator& communicator,
    Logger& logger)
  : locEstimator(locEstimator),
    attEstimator(attEstimator),
    raw1000MessageStream(communicator, 10),
    raw50MessageStream(communicator, 5),
    raw10MessageStream(communicator, 1),
    logger(logger) {
}

WorldEstimate WorldEstimator::update(const SensorMeasurements& meas) {
  WorldEstimate estimate;
  estimate.sensors = meas;
  estimate.att = attEstimator.update(meas);
  estimate.loc = locEstimator.update(meas, estimate.att);

  // TODO(yoos): Need RateLimitedStream for logger.
  static uint16_t i = 0;
  // Log 1 kHz stream
  if (i % 1 == 0) {
    protocol::message::raw_1000_message_t msg_1000 {
      .time = ST2MS(chibios_rt::System::getTime()),
        .accel  = {(*meas.accel).axes[0],
          (*meas.accel).axes[1],
          (*meas.accel).axes[2]},
        .accelH = {(*meas.accelH).axes[0],
          (*meas.accelH).axes[1],
          (*meas.accelH).axes[2]},
        .gyro   = {(*meas.gyro).axes[0],
          (*meas.gyro).axes[1],
          (*meas.gyro).axes[2]}
    };
    logger.write(msg_1000);
    if (raw1000MessageStream.ready()) {
      raw1000MessageStream.publish(msg_1000);
    }
  }

  // DEBUG
  //static int loop=0;
  //if (loop == 0) {
  //  BaseSequentialStream* chp = (BaseSequentialStream*)&SD4;
  //  chprintf(chp, "%d: %8f %8f %8f\r\n", chibios_rt::System::getTime(), (*meas.accelH).axes[0], (*meas.accelH).axes[1], (*meas.accelH).axes[2]);
  //}
  //loop = (loop+1) % 10;

  // Log 50 Hz stream
  if (i % 20 == 0) {
    protocol::message::raw_50_message_t msg_50 {
      .time = ST2MS(chibios_rt::System::getTime()),
      .bar  = (*meas.bar).pressure,
      .temp = (*meas.bar).temperature,
      .mag  = {(*meas.mag).axes[0],
               (*meas.mag).axes[1],
               (*meas.mag).axes[2]}
    };
    logger.write(msg_50);
    if (raw50MessageStream.ready()) {
      raw50MessageStream.publish(msg_50);
    }
  }

  // Log 10 Hz stream
  static uint16_t geigerCount = 0;
  geigerCount += (*meas.ggr).blips;
  if (i % 100 == 0) {
    protocol::message::raw_10_message_t msg_10 {
      .time = ST2MS(chibios_rt::System::getTime()),
      .gps_valid = (*meas.gps).valid,
      .lat  = (*meas.gps).lat,
      .lon  = (*meas.gps).lon,
      .utc  = (*meas.gps).utc,
      .geigerCount = geigerCount
    };
    logger.write(msg_10);
    if (raw10MessageStream.ready()) {
      raw10MessageStream.publish(msg_10);
    }
    geigerCount = 0;
  }
  i = (i+1) % 1000;

  return estimate;
}
