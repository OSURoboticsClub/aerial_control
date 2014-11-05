#include <platform_config.hpp>

namespace platform {

MPU6000 imu(&SPID1);
EsraRocketSystem system(&imu, &imu);

}
