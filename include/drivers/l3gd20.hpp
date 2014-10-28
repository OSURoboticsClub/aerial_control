#ifndef L3GD20_HPP_
#define L3GD20_HPP_

#include <hal.h>

#include <sensor/gyroscope.hpp>

#define L3GD20_SENSITIVITY_250DPS   (0.00875F)
#define L3GD20_SENSITIVITY_500DPS   (0.0175F)
#define L3GD20_SENSITIVITY_2000DPS  (0.070F)
#define L3GD20_DPS_TO_RADS          (0.017453293F)

#define L3GD20_SPI_DI               0xFF
#define L3GD20_SPI_DI_0             0x01
#define L3GD20_SPI_DI_1             0x02
#define L3GD20_SPI_DI_2             0x04
#define L3GD20_SPI_DI_3             0x08
#define L3GD20_SPI_DI_4             0x10
#define L3GD20_SPI_DI_5             0x20
#define L3GD20_SPI_DI_6             0x40
#define L3GD20_SPI_DI_7             0x80

#define L3GD20_SPI_AD               0x3F
#define L3GD20_SPI_AD_0             0x01
#define L3GD20_SPI_AD_1             0x02
#define L3GD20_SPI_AD_2             0x04
#define L3GD20_SPI_AD_3             0x08
#define L3GD20_SPI_AD_4             0x10
#define L3GD20_SPI_AD_5             0x20

#define L3GD20_SPI_MS               0x40
#define L3GD20_SPI_RW               0x80

#define L3GD20_SPI_AD_WHO_AM_I      0x0F
#define L3GD20_SPI_AD_CTRL_REG1     0x20
#define L3GD20_SPI_AD_CTRL_REG2     0x21
#define L3GD20_SPI_AD_CTRL_REG3     0x22
#define L3GD20_SPI_AD_CTRL_REG4     0x23
#define L3GD20_SPI_AD_CTRL_REG5     0x24
#define L3GD20_SPI_AD_REFERENCE     0x25
#define L3GD20_SPI_AD_OUT_TEMP      0x26
#define L3GD20_SPI_AD_STATUS_REG    0x27
#define L3GD20_SPI_AD_OUT_X_L       0x28
#define L3GD20_SPI_AD_OUT_X_H       0x29
#define L3GD20_SPI_AD_OUT_Y_L       0x2A
#define L3GD20_SPI_AD_OUT_Y_H       0x2B
#define L3GD20_SPI_AD_OUT_Z_L       0x2C
#define L3GD20_SPI_AD_OUT_Z_H       0x2D
#define L3GD20_SPI_AD_FIFO_CTRL_REG 0x2E
#define L3GD20_SPI_AD_FIFO_SRC_REG  0x2F
#define L3GD20_SPI_AD_INT1_CFG      0x30
#define L3GD20_SPI_AD_INT1_SRC      0x31
#define L3GD20_SPI_AD_INT1_TSH_XH   0x32
#define L3GD20_SPI_AD_INT1_TSH_XL   0x33
#define L3GD20_SPI_AD_INT1_TSH_YH   0x34
#define L3GD20_SPI_AD_INT1_TSH_YL   0x35
#define L3GD20_SPI_AD_INT1_TSH_ZH   0x36
#define L3GD20_SPI_AD_INT1_TSH_ZL   0x37
#define L3GD20_SPI_AD_INT1_DURATION 0x38

class L3GD20 : public Gyroscope {
public:
  L3GD20(SPIDriver *spi);

  void init();
  gyroscope_reading_t read();

private:
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t val);

  SPIDriver *spi;
};

#endif
