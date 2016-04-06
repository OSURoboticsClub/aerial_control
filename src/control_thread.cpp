#include "control_thread.hpp"

#include "heartbeat_thread.hpp"
#include "communication/communicator.hpp"
#include "filesystem/filesystem.hpp"
#include "filesystem/logger.hpp"
#include "params/parameter_repository.hpp"

#include "unit_config.hpp"
#include "variant/platform.hpp"
#include "variant/sdc_platform.hpp"
#include "variant/usart_platform.hpp"
#include "variant/unit.hpp"

msg_t ControlThread::main() {
  // Build and initialize the system
  Platform platform;
  platform.init();

  auto& primaryStream = platform.get<USARTPlatform>().getPrimaryStream();
  SDCDriver& sdcd = platform.get<SDCPlatform>().getSDCDriver();   // TODO(yoos): make optional

  ParameterRepository params;

  // Start the background threads
  static HeartbeatThread heartbeatThread(params);
  static Communicator communicator(primaryStream);
  static Logger logger(sdcd, communicator);

  heartbeatThread.start(LOWPRIO);
  communicator.start();
  logger.start();

  // Build the unit
  Unit unit(platform, communicator, logger);

  // Loop at a fixed rate forever
  // NOTE: If the deadline is ever missed then the loop will hang indefinitely.
  systime_t deadline = chibios_rt::System::getTime();
  while(true) {
    deadline += MS2ST(unit_config::DT * 1000);

    unit.getSystem().update();

    sleepUntil(deadline);
  }
}
