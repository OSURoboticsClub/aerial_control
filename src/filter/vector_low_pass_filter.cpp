#include "vector_low_pass_filter.hpp"

VectorLowPassFilter::VectorLowPassFilter(float breakFrequency)
  : VectorLowPassFilter(breakFrequency, Eigen::Vector3f::Zero()) {
}

VectorLowPassFilter::VectorLowPassFilter(float breakFrequency, Eigen::Vector3f value)
  : xFilter(breakFrequency, value(0)),
    yFilter(breakFrequency, value(1)),
    zFilter(breakFrequency, value(2)) {
}

Eigen::Vector3f VectorLowPassFilter::apply(float dt, Eigen::Vector3f x) {
  return Eigen::Vector3f(xFilter.apply(dt, x(0)),
                         yFilter.apply(dt, x(1)),
                         zFilter.apply(dt, x(2)));
}
