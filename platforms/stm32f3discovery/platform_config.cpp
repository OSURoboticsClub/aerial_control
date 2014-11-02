#include <platform_config.hpp>

namespace platform {

L3GD20 gyro(&SPID1);
LSM303DLHC accel(&I2CD1);
DefaultMultirotorVehicleSystem system(&accel, &gyro);

}
