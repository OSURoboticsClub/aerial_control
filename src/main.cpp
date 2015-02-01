#include "hal.h"
#include "ch.hpp"

#include "control_thread.hpp"
#include "parameter/parameter.hpp"
#include "parameter/parameter_store.hpp"

int main(void) {
  // Initialize ChibiOS.
  halInit();
  chibios_rt::System::init();

  // Start the main control thread.
  static ControlThread controlThread;
  controlThread.start(HIGHPRIO);

  // Wait for the control thread to exit. This should never happen.
  controlThread.wait();

  return 0;
}
