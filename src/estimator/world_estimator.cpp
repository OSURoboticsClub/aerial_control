#include "estimator/world_estimator.hpp"

#include <cstdio>
#include "hal.h"

#include "unit_config.hpp"
#include "util/time.hpp"

WorldEstimator::WorldEstimator(
    LocationEstimator& locEstimator,
    AttitudeEstimator& attEstimator,
    Communicator& communicator,
    Logger& logger)
  : locEstimator(locEstimator),
    attEstimator(attEstimator),
    worldMessageStream(communicator, 1),
    logger(logger) {
}

WorldEstimate WorldEstimator::update(const SensorMeasurements& meas) {
  WorldEstimate estimate {
    .sensors = meas,
    .loc = locEstimator.update(meas),
    .att = attEstimator.update(meas)
  };

  if(worldMessageStream.ready() && meas.gps) {
    // protocol::message::log_message_t m;
    // sprintf(m.data, "gps: %d %d", (int) (*meas.gps).lat * 1000, (int) (*meas.gps).lon * 1000);
    //
    // worldMessageStream.publish(m);
  }

  protocol::message::raw_1000_message_t msg_1000 {
    .time = ST2MS(chibios_rt::System::getTime()),
    .accel = {(*meas.accel).axes[0],
              (*meas.accel).axes[1],
              (*meas.accel).axes[2]},
    .accelH = {(*meas.accelH).axes[0],
               (*meas.accelH).axes[1],
               (*meas.accelH).axes[2]},
    .gyro = {(*meas.gyro).axes[0],
             (*meas.gyro).axes[1],
             (*meas.gyro).axes[2]}
  };
  logger.write(msg_1000);

  return estimate;
}
