#include <debugger.hpp>

Debugger::Debugger() : chibios_rt::BaseStaticThread<256>() {
}

msg_t Debugger::main() {
  setName("debug");

  int i = 0;
  while(true) {
    // TODO: chprintf is really slow at printing floats and causes us to
    // miss our deadline.
    // chprintf((BaseSequentialStream *) &SD1, "roll: %d pitch: %d yaw: %d\r\n",
    //     (int) estimate.roll, (int) estimate.pitch, (int) estimate.yaw);

    sleep(MS2ST(250));
  }

  return 0;
}
