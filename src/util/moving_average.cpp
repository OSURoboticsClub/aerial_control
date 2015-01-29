#include "util/moving_average.hpp"

void MovingAverage::add(float datum) {
  average = (datum + count * average) / (count + 1);

  ++count;
}

float MovingAverage::getAverage() {
  return average;
};
