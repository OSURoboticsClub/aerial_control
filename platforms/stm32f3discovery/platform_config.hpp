#ifndef PLATFORM_CONFIG_HPP_
#define PLATFORM_CONFIG_HPP_

// Drivers
#include <drivers/l3gd20.hpp>
#include <drivers/lsm303dlhc.hpp>

// Systems
#include <system/default_multirotor_vehicle_system.hpp>

namespace platform {

extern L3GD20 gyro;
extern LSM303DLHC accel;
extern DefaultMultirotorVehicleSystem system;

extern void init();

}

#endif // PLATFORM_CONFIG_HPP_
