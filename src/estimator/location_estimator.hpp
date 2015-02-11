#ifndef LOCATION_ESTIMATOR_HPP_
#define LOCATION_ESTIMATOR_HPP__

#include "sensors/sensor_measurements.hpp"

struct LocationEstimate {
  float lat;
  float lon;
  float altitude;
};

class LocationEstimator {
public:
  virtual LocationEstimate update(const SensorMeasurements& meas) = 0;
};

#endif
