#include <platform_config.hpp>

#include <hal_config.hpp>

namespace platform {

L3GD20 gyro(&SPID1, &l3gd20_spi_config);
LSM303DLHC accel(&I2CD1);
DefaultMultirotorVehicleSystem system(gyro, accel);

void init() {
  // Initialize platform HAL
  i2cPlatformInit();
  pwmPlatformInit();
  spiPlatformInit();
  usartPlatformInit();

  // Initialize platform devices
  gyro.init();
  accel.init();
  system.init();
}

}
