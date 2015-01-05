#include "ch.hpp"
#include "hal.h"

#include "communication.hpp"
#include "unit_config.hpp"
#include "variant/platform.hpp"
#include "variant/unit.hpp"

// static platform::HeartbeatThread heartbeatThread;
static CommunicationThread<255> communicationThread(
    reinterpret_cast<chibios_rt::BaseSequentialStreamInterface&>(SD1));

int main(void) {
  halInit();
  chibios_rt::System::init();

  Platform platform;
  Unit unit(platform);

  // Start the background threads
  // heartbeatThread.start(LOWPRIO);
  communicationThread.start(LOWPRIO);

  // Build and initialize the system
  platform.init();
  unit.init();

  // Loop at a fixed rate forever
  // NOTE: If the deadline is ever missed then the loop will hang indefinitely.
  systime_t deadline = chibios_rt::System::getTime();
  while(true) {
    deadline += MS2ST(DT * 1000);

    unit.getSystem().update();

    chibios_rt::BaseThread::sleepUntil(deadline);
  }

  return 0;
}
