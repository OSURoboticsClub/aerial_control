#include <ch.hpp>
#include <hal.h>

#include <config.hpp>

// Drivers
#include <drivers/l3gd20.hpp>
#include <drivers/lsm303dlhc.hpp>

// Misc
// #include <communicator.hpp>
#include <debugger.hpp>

// Systems
#include <system/default_multirotor_vehicle_system.hpp>

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

static HeartbeatThread heartbeatThread;
static Debugger debugger;

int main(void) {
  halInit();
  chibios_rt::System::init();

  // Start the background threads
  heartbeatThread.start(NORMALPRIO + 10);
  debugger.start(NORMALPRIO + 10);

  pwmInit();
  spiInit();
  i2cInit();

  // Start USART
  sdStart(&SD1, &usart1_config);
  palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));

  // Build and initialize the system
  L3GD20 gyro(&SPID1);
  LSM303DLHC accel(&I2CD1);

  gyro.init();
  accel.init();

  DefaultMultirotorVehicleSystem system(&accel, &gyro);
  system.init();

  // Loop at a fixed rate forever
  // NOTE: If the deadline is ever missed then the loop will hang indefinitely.
  systime_t deadline = chTimeNow();
  while(true) {
    deadline += MS2ST(DT * 1000);

    system.update();

    chibios_rt::BaseThread::sleepUntil(deadline);
 }

  return 0;
}
