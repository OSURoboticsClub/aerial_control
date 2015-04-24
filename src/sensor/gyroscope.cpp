#include "sensor/gyroscope.hpp"

Gyroscope::Gyroscope() {
  clearGyrOffsets();
}

void Gyroscope::setGyrOffsets(std::array<float, 3> newOffsets) {
  gyrOffsets = {newOffsets[0], newOffsets[1], newOffsets[2]};
}

void Gyroscope::clearGyrOffsets() {
  gyrOffsets = {0,0,0};
}
