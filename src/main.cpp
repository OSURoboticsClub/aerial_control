#include "hal.h"
#include "ch.hpp"

#include "control_thread.hpp"

int main(void) {
  halInit();
  chibios_rt::System::init();

  static ControlThread controlThread;
  controlThread.start(HIGHPRIO);
  controlThread.wait();

  return 0;
}
