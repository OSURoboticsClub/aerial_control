#ifndef SENSOR_HPP_
#define SENSOR_HPP_

#include <array>

class Sensor {
public:
  Sensor();
  virtual bool healthy() = 0;

  void setAxisConfig(std::array<int, 3> newAxisConfig);
  void setOffsets(std::array<float, 3> newOffsets);

protected:
  std::array<int, 3> axes;
  std::array<int, 3> signs;
  std::array<float, 3> offsets;
};

#endif // SENSOR_HPP_
