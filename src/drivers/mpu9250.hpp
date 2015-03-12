#ifndef MPU9250_HPP_
#define MPU9250_HPP_

#include <cstdint>

#include "hal.h"

#include "drivers/spi_device.hpp"
#include "sensor/gyroscope.hpp"
#include "sensor/accelerometer.hpp"
#include "sensor/magnetometer.hpp"

namespace mpu9250 {

// TODO(yoos): Copied from MPU6000. Need to verify.
const std::uint8_t AUX_VDDIO          = 0x01;   // R/W
const std::uint8_t SMPLRT_DIV         = 0x19;   // R/W
const std::uint8_t CONFIG             = 0x1A;   // R/W
const std::uint8_t GYRO_CONFIG        = 0x1B;   // R/W
const std::uint8_t ACCEL_CONFIG       = 0x1C;   // R/W
const std::uint8_t FF_THR             = 0x1D;   // R/W
const std::uint8_t FF_DUR             = 0x1E;   // R/W
const std::uint8_t MOT_THR            = 0x1F;   // R/W
const std::uint8_t MOT_DUR            = 0x20;   // R/W
const std::uint8_t ZRMOT_THR          = 0x21;   // R/W
const std::uint8_t ZRMOT_DUR          = 0x22;   // R/W
const std::uint8_t FIFO_EN            = 0x23;   // R/W
const std::uint8_t I2C_MST_CTRL       = 0x24;   // R/W
const std::uint8_t I2C_SLV0_ADDR      = 0x25;   // R/W
const std::uint8_t I2C_SLV0_REG       = 0x26;   // R/W
const std::uint8_t I2C_SLV0_CTRL      = 0x27;   // R/W
const std::uint8_t I2C_SLV1_ADDR      = 0x28;   // R/W
const std::uint8_t I2C_SLV1_REG       = 0x29;   // R/W
const std::uint8_t I2C_SLV1_CTRL      = 0x2A;   // R/W
const std::uint8_t I2C_SLV2_ADDR      = 0x2B;   // R/W
const std::uint8_t I2C_SLV2_REG       = 0x2C;   // R/W
const std::uint8_t I2C_SLV2_CTRL      = 0x2D;   // R/W
const std::uint8_t I2C_SLV3_ADDR      = 0x2E;   // R/W
const std::uint8_t I2C_SLV3_REG       = 0x2F;   // R/W
const std::uint8_t I2C_SLV3_CTRL      = 0x30;   // R/W
const std::uint8_t I2C_SLV4_ADDR      = 0x31;   // R/W
const std::uint8_t I2C_SLV4_REG       = 0x32;   // R/W
const std::uint8_t I2C_SLV4_DO        = 0x33;   // R/W
const std::uint8_t I2C_SLV4_CTRL      = 0x34;   // R/W
const std::uint8_t I2C_SLV4_DI        = 0x35;   // R
const std::uint8_t I2C_MST_STATUS     = 0x36;   // R
const std::uint8_t INT_PIN_CFG        = 0x37;   // R/W
const std::uint8_t INT_ENABLE         = 0x38;   // R/W
const std::uint8_t INT_STATUS         = 0x3A;   // R
const std::uint8_t ACCEL_XOUT_H       = 0x3B;   // R
const std::uint8_t ACCEL_XOUT_L       = 0x3C;   // R
const std::uint8_t ACCEL_YOUT_H       = 0x3D;   // R
const std::uint8_t ACCEL_YOUT_L       = 0x3E;   // R
const std::uint8_t ACCEL_ZOUT_H       = 0x3F;   // R
const std::uint8_t ACCEL_ZOUT_L       = 0x40;   // R
const std::uint8_t TEMP_OUT_H         = 0x41;   // R
const std::uint8_t TEMP_OUT_L         = 0x42;   // R
const std::uint8_t GYRO_XOUT_H        = 0x43;   // R
const std::uint8_t GYRO_XOUT_L        = 0x44;   // R
const std::uint8_t GYRO_YOUT_H        = 0x45;   // R
const std::uint8_t GYRO_YOUT_L        = 0x46;   // R
const std::uint8_t GYRO_ZOUT_H        = 0x47;   // R
const std::uint8_t GYRO_ZOUT_L        = 0x48;   // R
const std::uint8_t EXT_SENS_DATA_00   = 0x49;   // R
const std::uint8_t EXT_SENS_DATA_01   = 0x4A;   // R
const std::uint8_t EXT_SENS_DATA_02   = 0x4B;   // R
const std::uint8_t EXT_SENS_DATA_03   = 0x4C;   // R
const std::uint8_t EXT_SENS_DATA_04   = 0x4D;   // R
const std::uint8_t EXT_SENS_DATA_05   = 0x4E;   // R
const std::uint8_t EXT_SENS_DATA_06   = 0x4F;   // R
const std::uint8_t EXT_SENS_DATA_07   = 0x50;   // R
const std::uint8_t EXT_SENS_DATA_08   = 0x51;   // R
const std::uint8_t EXT_SENS_DATA_09   = 0x52;   // R
const std::uint8_t EXT_SENS_DATA_10   = 0x53;   // R
const std::uint8_t EXT_SENS_DATA_11   = 0x54;   // R
const std::uint8_t EXT_SENS_DATA_12   = 0x55;   // R
const std::uint8_t EXT_SENS_DATA_13   = 0x56;   // R
const std::uint8_t EXT_SENS_DATA_14   = 0x57;   // R
const std::uint8_t EXT_SENS_DATA_15   = 0x58;   // R
const std::uint8_t EXT_SENS_DATA_16   = 0x59;   // R
const std::uint8_t EXT_SENS_DATA_17   = 0x5A;   // R
const std::uint8_t EXT_SENS_DATA_18   = 0x5B;   // R
const std::uint8_t EXT_SENS_DATA_19   = 0x5C;   // R
const std::uint8_t EXT_SENS_DATA_20   = 0x5D;   // R
const std::uint8_t EXT_SENS_DATA_21   = 0x5E;   // R
const std::uint8_t EXT_SENS_DATA_22   = 0x5F;   // R
const std::uint8_t EXT_SENS_DATA_23   = 0x60;   // R
const std::uint8_t MOT_DETECT_STATUS  = 0x61;   // R
const std::uint8_t I2C_SLV0_DO        = 0x63;   // R/W
const std::uint8_t I2C_SLV1_DO        = 0x64;   // R/W
const std::uint8_t I2C_SLV2_DO        = 0x65;   // R/W
const std::uint8_t I2C_SLV3_DO        = 0x66;   // R/W
const std::uint8_t I2C_MST_DELAY_CTRL = 0x67;   // R/W
const std::uint8_t SIGNAL_PATH_RESET  = 0x68;   // R/W
const std::uint8_t MOT_DETECT_CTRL    = 0x69;   // R/W
const std::uint8_t USER_CTRL          = 0x6A;   // R/W
const std::uint8_t PWR_MGMT_1         = 0x6B;   // R/W
const std::uint8_t PWR_MGMT_2         = 0x6C;   // R/W
const std::uint8_t FIFO_COUNTH        = 0x72;   // R/W
const std::uint8_t FIFO_COUNTL        = 0x73;   // R/W
const std::uint8_t FIFO_R_W           = 0x74;   // R/W
const std::uint8_t WHO_AM_I           = 0x75;   // R

}

// TODO(yoos): Inherit magnetometer
class MPU9250 : protected SPIDevice<8, 8>, public Gyroscope, public Accelerometer, public Magnetometer {
public:
  using SPIDevice::SPIDevice;

  void init() override;
  GyroscopeReading readGyro() override;
  AccelerometerReading readAccel() override;
  MagnetometerReading readMag() override;

private:
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t val);
};

#endif
