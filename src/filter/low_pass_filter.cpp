#include "low_pass_filter.hpp"

#include "util/math.hpp"

LowPassFilter::LowPassFilter(float breakFrequency) : LowPassFilter(breakFrequency, 0.0) {
}

LowPassFilter::LowPassFilter(float breakFrequency, float value) : breakFrequency(breakFrequency), value(value) {
}

float LowPassFilter::apply(float dt, float x) {
  float rc = 1.0 / (2.0 * M_PI * breakFrequency);
  float alpha = dt / (dt + rc);
  value = alpha * x + (1.0 - alpha) * value;

  return value;
}
