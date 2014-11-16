#ifndef PLATFORM_CONFIG_HPP_
#define PLATFORM_CONFIG_HPP_

#include <ch.hpp>

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

class HeartbeatThread : public chibios_rt::BaseStaticThread<64> {
public:
  HeartbeatThread() : chibios_rt::BaseStaticThread<64>() {
  }

  virtual msg_t main() {
    while(true) {
      palSetPad(GPIOE, GPIOE_LED3_RED);
      sleep(MS2ST(500));
      palClearPad(GPIOE, GPIOE_LED3_RED);
      sleep(MS2ST(500));
    }

    return 0;
  }
};

}

#endif // PLATFORM_CONFIG_HPP_
