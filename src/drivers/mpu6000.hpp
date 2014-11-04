#ifndef MPU6000_HPP_
#define MPU6000_HPP_

#include <hal.h>

#include <sensor/imu.hpp>

class MPU6000 : public IMU {
public:
  MPU6000(SPIDriver *spi);

  void init() override;
  imu_reading_t read() override;

private:
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t val);

  SPIDriver *spi;
};

#endif
