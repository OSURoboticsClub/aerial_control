#include "ch.hpp"
#include "hal.h"

#include "communication/communicator.hpp"
#include "communication/communication_thread.hpp"

#include "unit_config.hpp"
#include "variant/platform.hpp"
#include "variant/usart_platform.hpp"
#include "variant/unit.hpp"

class HeartbeatThread : public chibios_rt::BaseStaticThread<64> {
public:
  HeartbeatThread() : chibios_rt::BaseStaticThread<64>() {
  }

  virtual msg_t main() {
    while(true) {
      // palSetPad(GPIOE, GPIOE_LED3_RED);
      sleep(MS2ST(500));
      // palClearPad(GPIOE, GPIOE_LED3_RED);
      sleep(MS2ST(500));
    }
    return 0;
  }
};


int main(void) {
  halInit();
  chibios_rt::System::init();

  // TODO: This seems? to fix crashing issues on OSUAR rev. 3. Is this actually
  // the case?
  chibios_rt::BaseThread::sleep(MS2ST(100));

  // Build and initialize the system
  Platform platform;
  platform.init();

  auto& primaryStream = platform.get<USARTPlatform>().getPrimaryStream();
  static Communicator communicator(primaryStream);

  Unit unit(platform, communicator);
  unit.init();

  // Start the background threads
  static HeartbeatThread heartbeatThread;
  static CommunicationThread communicationThread(communicator);

  heartbeatThread.start(LOWPRIO);
  communicationThread.start(LOWPRIO);

  // Loop at a fixed rate forever
  // NOTE: If the deadline is ever missed then the loop will hang indefinitely.
  systime_t deadline = chibios_rt::System::getTime();
  while(true) {
    deadline += MS2ST(unit_config::DT * 1000);

    unit.getSystem().update();

    chibios_rt::BaseThread::sleepUntil(deadline);
  }

  return 0;
}
