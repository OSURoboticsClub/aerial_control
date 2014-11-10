#ifndef PLATFORM_CONFIG_HPP_
#define PLATFORM_CONFIG_HPP_

#include <ch.hpp>

// Drivers
#include <drivers/mpu6000.hpp>

// Systems
#include <system/default_multirotor_vehicle_system.hpp>

namespace platform {

extern MPU6000 imu;
extern DefaultMultirotorVehicleSystem system;

extern void init();

class HeartbeatThread : public chibios_rt::BaseStaticThread<64> {
public:
  HeartbeatThread() : chibios_rt::BaseStaticThread<64>(){};
  virtual msg_t main(void);
};

}

#endif // PLATFORM_CONFIG_HPP_
