#ifndef LOCATION_ESTIMATOR_HPP_
#define LOCATION_ESTIMATOR_HPP_

#include "estimator/attitude_estimator.hpp"
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
  virtual LocationEstimate update(const SensorMeasurements& meas, const AttitudeEstimate& att) = 0;
};

#endif
