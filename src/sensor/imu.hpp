#ifndef IMU_HPP_
#define IMU_HPP_

struct imu_reading_t {
  float gyr[3];
  float acc[3];
};

class IMU {
public:
  virtual void init() =0;
  virtual imu_reading_t read() =0;
};

#endif
