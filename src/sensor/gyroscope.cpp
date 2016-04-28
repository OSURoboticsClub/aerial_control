#include "sensor/gyroscope.hpp"

#include <cmath>

void Gyroscope::calibrateStep() {
  // Don't continue to average if acceptable offsets have already been computed.
  if (calibrated()) {
    return;
  }

  GyroscopeReading reading = readGyro();

  static std::array<float, 3> calibOffsets = {0,0,0};
  for (std::size_t i = 0; i < 3; i++) {
    calibOffsets[i] = (calibOffsets[i] * calibrationCount + reading.axes[i] + offsets[i]) / (calibrationCount + 1);
  }

  calibrationCount++;

  if (std::abs(reading.axes[0] > 0.1) || std::abs(reading.axes[1] > 0.1) ||
      std::abs(reading.axes[2] > 0.1)) {
    calibrationCount = 0;
  }

  if (calibrated()) {
    for (std::size_t i=0; i<3; i++) {
      offsets[i] = calibOffsets[i];
    }
  }
}

bool Gyroscope::calibrated() const {
  return calibrationCount > 5000;
}
