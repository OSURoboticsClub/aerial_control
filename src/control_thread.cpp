#include "control_thread.hpp"

#include "heartbeat_thread.hpp"
#include "communication/communicator.hpp"
#include "communication/communication_thread.hpp"

#include "unit_config.hpp"
#include "variant/platform.hpp"
#include "variant/usart_platform.hpp"
#include "variant/unit.hpp"

msg_t ControlThread::main() {
  // Build and initialize the system
  Platform platform;
  platform.init();

  auto& primaryStream = platform.get<USARTPlatform>().getPrimaryStream();
  static Communicator communicator(primaryStream);

  Unit unit(platform, communicator);

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

    sleepUntil(deadline);
  }
}
