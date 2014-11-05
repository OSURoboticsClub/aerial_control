#ifndef ACCELEROMETER_HPP_
#define ACCELEROMETER_HPP_

struct accelerometer_reading_t {
  float axes[3];
};

class Accelerometer {
public:
  virtual void init() =0;
  virtual accelerometer_reading_t readAccel() =0;
};

#endif
