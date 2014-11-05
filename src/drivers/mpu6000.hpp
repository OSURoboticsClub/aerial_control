#ifndef MPU6000_HPP_
#define MPU6000_HPP_

#include <hal.h>

#include <sensor/accelerometer.hpp>
#include <sensor/gyroscope.hpp>

class MPU6000 : public Accelerometer, public Gyroscope {
public:
  MPU6000(SPIDriver *spi);

  void init() override;
  accelerometer_reading_t readAccel() override;
  gyroscope_reading_t readGyro() override;

private:
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t val);

  SPIDriver *spi;
};

#endif
