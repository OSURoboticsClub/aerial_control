#include <platform_config.hpp>

#include <hal_config.hpp>

namespace platform {

L3GD20 gyro(&SPID1);
LSM303DLHC accel(&I2CD1);
DefaultMultirotorVehicleSystem system(&accel, &gyro);

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
