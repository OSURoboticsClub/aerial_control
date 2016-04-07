#ifndef VECTOR_LOW_PASS_FILTER_H_
#define VECTOR_LOW_PASS_FILTER_H_

#include "Eigen/Dense"

#include "filter/low_pass_filter.hpp"

class VectorLowPassFilter {
public:
  VectorLowPassFilter(float breakFrequency);
  VectorLowPassFilter(float breakFrequency, Eigen::Vector3f value);

  Eigen::Vector3f apply(float dt, Eigen::Vector3f x);

private:
  LowPassFilter xFilter;
  LowPassFilter yFilter;
  LowPassFilter zFilter;
};

#endif // VECTOR_LOW_PASS_FILTER_H_
