#include <platform_config.hpp>

#include <hal_config.hpp>

namespace platform {

MPU6000 imu(&SPID1, &mpu6000_spicfg);
DefaultMultirotorVehicleSystem system(&imu, &imu);

void init() {
  // Initialize platform HAL
  i2cPlatformInit();
  pwmPlatformInit();
  spiPlatformInit();
  usartPlatformInit();

  // Initialize platform devices
  imu.init();
  system.init();

  palSetPadMode(GPIOA, 6, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA, 7, PAL_MODE_OUTPUT_PUSHPULL);
}

msg_t HeartbeatThread::main(void) {
  while(true) {
    palSetPad(GPIOA, 6);
    sleep(MS2ST(50));
    palClearPad(GPIOA, 6);
    palSetPad(GPIOA, 7);
    sleep(MS2ST(50));
    palClearPad(GPIOA, 7);
    sleep(MS2ST(900));
  }

  return 0;
}

}
