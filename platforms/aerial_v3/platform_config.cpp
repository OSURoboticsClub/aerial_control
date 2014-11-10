#include <platform_config.hpp>

namespace platform {

MPU6000 imu(&SPID1, &mpu6000_spicfg);
DefaultMultirotorVehicleSystem system(&imu, &imu);

void init() {
  imu.init();
  system.init();

  palSetPadMode(GPIOA, 6, PAL_MODE_OUTPUT_PUSHPULL);
}

msg_t HeartbeatThread::main(void) {
  while(true) {
    palSetPad(GPIOA, 6);
    sleep(MS2ST(50));
    palClearPad(GPIOA, 6);
    sleep(MS2ST(950));
  }

  return 0;
}

}
