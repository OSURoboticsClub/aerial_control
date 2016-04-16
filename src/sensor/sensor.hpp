#ifndef SENSOR_HPP_
#define SENSOR_HPP_

#include <array>

template <int axes_count = 3>
class Sensor {
public:
  Sensor();

  /**
   * Performs a single step in the calibration process for this sensor. The
   * calibration process is composed of any number of calls to `calibrateStep`,
   * as dictated by how long it takes for the `calibrated` function to return
   * true.
   *
   * That is, a sensor continually attempts to refine its calibration until it
   * is within tolerance. Then, `calibrated` shall return true.
   *
   * The default implementation does nothing.
   */
  virtual void calibrateStep() {
    // Default do-nothing impl
  }

  /**
   * Returns whether or not the sensor calibration has completed.
   *
   * The default implemention always returns true.
   *
   * @see #calibrateStep()
   */
  virtual bool calibrated() const {
    // Default do-nothing impl
    return true;
  }

  virtual bool healthy() = 0;

  /**
   * Configure sensor axes.
   *
   * Example input of {3, -1, 2} here will interpret the chip's X axis as Z,
   * Y axis as negative X, and Z as Y.
   */
  void setAxisConfig(const std::array<int, axes_count>& newAxisConfig);

  void setOffsets(const std::array<float, axes_count>& newOffsets);

  const std::array<float, axes_count>& getOffsets() const;

protected:
  std::array<int, axes_count> axes;
  std::array<int, axes_count> signs;
  std::array<float, axes_count> offsets;
};

#include "sensor/sensor.tpp"

#endif // SENSOR_HPP_
