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

    protocol::message::imu_message_t msg_imu {
      .time = ST2MS(chibios_rt::System::getTime())
    };
    logger.write(msg_imu);
  }

  return estimate;
}
