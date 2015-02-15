#ifndef LOCATION_ESTIMATOR_HPP_
#define LOCATION_ESTIMATOR_HPP_

#include "sensor/sensor_measurements.hpp"

struct LocationEstimate {
  float lat;
  float lon;
  float alt;
};

class LocationEstimator {
public:
  virtual LocationEstimate update(const SensorMeasurements& meas) = 0;
};

#endif
