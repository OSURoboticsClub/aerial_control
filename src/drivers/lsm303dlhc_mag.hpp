#ifndef LSM303DLHC_MAG_HPP_
#define LSM303DLHC_MAG_HPP_

#include <cstdint>

#include "hal.h"

#include "drivers/i2c_device.hpp"
#include "sensor/magnetometer.hpp"

namespace lsm303dlhc_mag {

const std::uint8_t I2C_CRA_REG_M    = 0x00;
const std::uint8_t I2C_CRB_REG_M    = 0x01;
const std::uint8_t I2C_MR_REG_M     = 0x02;
const std::uint8_t I2C_OUT_X_H_M    = 0x03;
const std::uint8_t I2C_OUT_X_L_M    = 0x04;
const std::uint8_t I2C_OUT_Z_H_M    = 0x05;
const std::uint8_t I2C_OUT_Z_L_M    = 0x06;
const std::uint8_t I2C_OUT_Y_H_M    = 0x07;
const std::uint8_t I2C_OUT_Y_L_M    = 0x08;
const std::uint8_t I2C_SR_REG_MG    = 0x09;
const std::uint8_t I2C_IRA_REG_M    = 0x0A;
const std::uint8_t I2C_IRB_REG_M    = 0x0B;
const std::uint8_t I2C_IRC_REG_M    = 0x0C;
const std::uint8_t I2C_TEMP_OUT_H_M = 0x31;
const std::uint8_t I2C_TEMP_OUT_L_M = 0x32;

}

class LSM303DLHCMag : protected I2CDevice<8, 8>, public Magnetometer {
public:
  using I2CDevice::I2CDevice;

  void init() override;
  bool isHealthy() override;
  MagnetometerReading readMag() override;

private:
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t val);
};


#endif /* LSM303DLHC_MAG_HPP_ */
