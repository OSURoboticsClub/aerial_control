#ifndef LOCATION_ESTIMATOR_HPP_
#define LOCATION_ESTIMATOR_HPP_

#include "sensor/sensor_measurements.hpp"

struct LocationEstimate {
  uint32_t time;
  float lat;
  float lon;
  float alt;
  float dAlt;
  std::array<float, 3> jerk;
};

class LocationEstimator {
public:
  virtual LocationEstimate update(const SensorMeasurements& meas) = 0;
};

#endif
