#include "control_thread.hpp"

#include "heartbeat_thread.hpp"
#include "communication/communicator.hpp"

#include "unit_config.hpp"
#include "variant/platform.hpp"
#include "variant/usart_platform.hpp"
#include "variant/unit.hpp"

msg_t ControlThread::main() {
  // Build and initialize the system
  Platform platform;
  platform.init();

  auto& primaryStream = platform.get<USARTPlatform>().getPrimaryStream();

  // Start the background threads
  static HeartbeatThread heartbeatThread;
  static Communicator communicator(primaryStream);

  heartbeatThread.start(LOWPRIO);
  communicator.start();

  // Build the unit
  Unit unit(platform, communicator);

  // Loop at a fixed rate forever
  // NOTE: If the deadline is ever missed then the loop will hang indefinitely.
  systime_t deadline = chibios_rt::System::getTime();
  while(true) {
    float dt = unit.getParams().find<float>("DT")->get();

    deadline += MS2ST(dt * 1000);

    unit.getSystem().update();

    sleepUntil(deadline);
  }
}
