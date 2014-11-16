#include <platform_config.hpp>

namespace platform {

MPU6000 imu(&SPID1, &mpu6000_spicfg);
EsraRocketSystem system(&imu, &imu);

void init() {
  imu.init();
  system.init();
}

}
