#include "sensor/sensor.hpp"

Sensor::Sensor() {
  // Default to device axes and zero offsets.
  setAxisConfig({1,2,3});
  setOffsets({0,0,0});
}

void Sensor::setAxisConfig(std::array<int, 3> newAxisConfig) {
  for (int i=0; i<3; i++) {
    signs[i] = (newAxisConfig[i] > 0) ? 1 : -1;
    axes[i] = signs[i] * newAxisConfig[i] - 1;
  }
}

void Sensor::setOffsets(std::array<float, 3> newOffsets) {
  for (int i=0; i<3; i++) {
    offsets[i] = newOffsets[i];
  }
}
