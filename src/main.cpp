#include "hal.h"
#include "ch.hpp"

#include "parameter.hpp"
#include "control_thread.hpp"

int main(void) {
  // Initialize ChibiOS.
  halInit();
  chibios_rt::System::init();

  // TODO: Parameter test code >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  Parameter<int> myParam("MY_PARAM", 5);

  ParameterStore store;
  store.insert(&myParam);

  store.find("MY_PARAM");
  // TODO: Parameter test code <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

  // Start the main control thread.
  static ControlThread controlThread;
  controlThread.start(HIGHPRIO);

  // Wait for the control thread to exit. This should never happen.
  controlThread.wait();

  return 0;
}
