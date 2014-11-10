#include <ch.hpp>
#include <hal.h>

#include <hal_config.hpp>
#include <platform_config.hpp>

// Misc
// #include <communicator.hpp>
#include <debugger.hpp>

static platform::HeartbeatThread heartbeatThread;
static Debugger debugger;

int main(void) {
  halInit();
  chibios_rt::System::init();

  // Start the background threads
  heartbeatThread.start(LOWPRIO);
  debugger.start(LOWPRIO);

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
