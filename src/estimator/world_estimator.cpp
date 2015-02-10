#include "estimator/world_estimator.hpp"

#include <cstdio>
#include "hal.h"

#include "unit_config.hpp"

WorldEstimator::WorldEstimator(Communicator& communicator)
  : worldMessageStream(communicator, 10) {
}

WorldEstimate WorldEstimator::update(const SensorReadingGroup& readings) {
  // TODO: get GPS data

  WorldEstimate estimate {
    .globe_loc = {1, 2, 3}
  };

  if(worldMessageStream.ready()) {
    protocol::message::log_message_t m;
    sprintf(m.data, "world estimate test");

    worldMessageStream.publish(m);
  }

  return estimate;
}
