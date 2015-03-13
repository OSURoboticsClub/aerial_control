#ifndef GYROSCOPE_HPP_
#define GYROSCOPE_HPP_

#include <array>

struct GyroscopeReading {
  std::array<float, 3> axes;
};

class Gyroscope {
public:
  virtual void init() = 0;
  virtual bool isHealthy() = 0;
  virtual GyroscopeReading readGyro() = 0;
};

#endif
