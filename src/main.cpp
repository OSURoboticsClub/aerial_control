#include <ch.hpp>
#include <hal.h>

#include <hal_config.hpp>
#include <platform_config.hpp>

// Misc
// #include <communicator.hpp>
#include <debugger.hpp>

/**
 * Blinks an LED to show that the software is still alive.
 */
class HeartbeatThread : public chibios_rt::BaseStaticThread<64> {
public:
  HeartbeatThread() : chibios_rt::BaseStaticThread<64>() {
  }

  virtual msg_t main() {
    while(true) {
      //palSetPad(GPIOE, GPIOE_LED3_RED);
      sleep(MS2ST(500));
      //palClearPad(GPIOE, GPIOE_LED3_RED);
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
  heartbeatThread.start(LOWPRIO);
  debugger.start(LOWPRIO);

  pwmInit();
  spiInit();
  i2cInit();

  i2cPlatformInit();
  pwmPlatformInit();
  spiPlatformInit();
  usartPlatformInit();

  // Build and initialize the system
  platform::init();

  // Loop at a fixed rate forever
  // NOTE: If the deadline is ever missed then the loop will hang indefinitely.
  systime_t deadline = chTimeNow();
  while(true) {
    deadline += MS2ST(DT * 1000);

    platform::system.update();

    chibios_rt::BaseThread::sleepUntil(deadline);
  }

  return 0;
}
