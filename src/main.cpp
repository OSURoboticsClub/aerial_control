#include <ch.hpp>
#include <hal.h>

#include <config.hpp>
#include <platform_config.hpp>

// Misc
// #include <communicator.hpp>
#include <debugger.hpp>

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

  usartPlatformInit();

  // Build and initialize the system
  platform::gyro.init();
  platform::accel.init();
  platform::system.init();

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
