#include "sensor/accelerometer.hpp"

Accelerometer::Accelerometer() {
  clearAccOffsets();
}

void Accelerometer::setAccOffsets(std::array<float, 3> newOffsets) {
  accOffsets[0] = newOffsets[0];
  accOffsets[1] = newOffsets[1];
  accOffsets[2] = newOffsets[2];
}

void Accelerometer::clearAccOffsets() {
  accOffsets = {0,0,0};
}
