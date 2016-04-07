#include "low_pass_filter.hpp"

#include "util/math.hpp"

LowPassFilter::LowPassFilter(float breakFrequency) : LowPassFilter(breakFrequency, 0.0) {
}

LowPassFilter::LowPassFilter(float breakFrequency, float value) : breakFrequency(breakFrequency), value(value) {
}

float LowPassFilter::apply(float dt, float x) {
  float omega = 2.0 * M_PI * breakFrequency;
  float alpha = omega / (omega + 1);
  value = alpha * x + (1.0 - alpha) * value;

  return value;
}
