#ifndef PLATFORM_CONFIG_HPP_
#define PLATFORM_CONFIG_HPP_

// Drivers
#include <drivers/mpu6000.hpp>

// Systems
#include <system/esra_rocket_system.hpp>

namespace platform {

extern MPU6000 imu;
extern EsraRocketSystem system;

extern void init();

}

#endif // PLATFORM_CONFIG_HPP_
