#include "control_thread.hpp"

#include "heartbeat_thread.hpp"
#include "communication/communicator.hpp"
#include "filesystem/filesystem.hpp"
#include "filesystem/logger.hpp"
#include "logger/logged_variable.hpp"

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

  // Start the background threads
  static HeartbeatThread heartbeatThread;
  //static Registry registry();   // TODO(syoo): merge #46

  // Set up communicator
  static Communicator communicator(primaryStream);
  //communicator.subscribe(registry);   // TODO(syoo): merge #46

  #if USE_FILESYSTEM==TRUE
  SDCDriver& sdcd = platform.get<SDCPlatform>().getSDCDriver();
  static Logger logger(platform);
  //logger.subscribe(registry);   // TODO(syoo): merge #46
  #endif

  heartbeatThread.start(LOWPRIO);
  //registry.start();
  communicator.start();
  logger.start(HIGHPRIO-1);

  // Build the unit
  Unit unit(platform, communicator, logger);

  // TODO: Test variable to ensure logging infrastructure is functioning.
  LoggedVariableRegistry registry;
  LoggedVariable<int> loopCounter(registry, 0);

  // Loop at a fixed rate forever
  // NOTE: If the deadline is ever missed then the loop will hang indefinitely.
  systime_t deadline = chibios_rt::System::getTime();
  while(true) {
    deadline += MS2ST(unit_config::DT * 1000);

    loopCounter.set(loopCounter.v() + 1);
    unit.getSystem().update();

    sleepUntil(deadline);
  }
}
