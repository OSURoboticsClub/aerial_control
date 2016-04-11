#include "control_thread.hpp"

#include "global_parameters.hpp"
#include "heartbeat_thread.hpp"
#include "communication/communicator.hpp"
#include "filesystem/filesystem.hpp"
#include "filesystem/logger.hpp"
#include "params/parameter_repository.hpp"
#include "variable_registry/variable.hpp"

#include "variant/platform.hpp"
#include "variant/sdc_platform.hpp"
#include "variant/usart_platform.hpp"
#include "variant/unit.hpp"

msg_t ControlThread::main() {
  // Build and initialize the system
  Platform platform;
  platform.init();

  auto& primaryStream = platform.get<USARTPlatform>().getPrimaryStream();

  ParameterRepository params;
  GlobalParameters globalParams(params);

  // Start the background threads
  static HeartbeatThread heartbeatThread(params);
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
  Unit unit(platform, params, communicator, logger);

  // TODO: Test variable to ensure logging infrastructure is functioning.
  VariableRegistry registry;
  RegisteredVariable<int> loopCounter(registry, 0);

  // Loop at a fixed rate forever
  // NOTE: If the deadline is ever missed then the loop will hang indefinitely.
  systime_t deadline = chibios_rt::System::getTime();
  while(true) {
    deadline += MS2ST(params.get(GlobalParameters::PARAM_DT) * 1000);

    loopCounter.set(loopCounter.v() + 1);
    unit.getSystem().update();

    sleepUntil(deadline);
  }
}
