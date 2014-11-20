#ifndef GYROSCOPE_HPP_
#define GYROSCOPE_HPP_

struct gyroscope_reading_t {
  float axes[3];
};

class Gyroscope {
public:
  virtual void init() = 0;
  virtual gyroscope_reading_t readGyro() = 0;
};

#endif
