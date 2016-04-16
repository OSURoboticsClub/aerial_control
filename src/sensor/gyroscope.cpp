#include "sensor/gyroscope.hpp"

#include <cmath>

void Gyroscope::calibrateStep() {
  if (calibrated()) {
    return;
  }

  GyroscopeReading reading = readGyro();

  for (std::size_t i = 0; i < 3; i++) {
    offsets[i] = (offsets[i] * calibrationCount + reading.axes[i]) /
                 (calibrationCount + 1);
  }

  calibrationCount++;

  if (std::abs(reading.axes[0] > 0.1) || std::abs(reading.axes[1] > 0.1) ||
      std::abs(reading.axes[2] > 0.1)) {
    calibrationCount = 0;
  }
}

bool Gyroscope::calibrated() const {
  return calibrationCount > 5000;
}
