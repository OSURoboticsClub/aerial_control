#ifndef GYROSCOPE_HPP_
#define GYROSCOPE_HPP_

struct gyroscope_reading_t {
  float axes[3];
};

class Gyroscope {
public:
  virtual void init() =0;
  virtual struct gyroscope_reading_t read() =0;
};

#endif
