#include "estimator/world_estimator.hpp"

#include <cstdio>
#include "hal.h"

#include "unit_config.hpp"

WorldEstimator::WorldEstimator(Communicator& communicator)
  : worldMessageStream(communicator, 10) {
}

world_estimate_t WorldEstimator::update(gps_reading_t& gpsReading) {
  // TODO: get GPS data

  world_estimate_t estimate {
    .globe_loc = {1, 2, 3}
  };

  if(worldMessageStream.ready()) {
    protocol::message::log_message_t m;
    sprintf(m.data, "world estimate test");

    worldMessageStream.publish(m);
  }

  return estimate;
}
