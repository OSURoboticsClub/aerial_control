#include "heartbeat_thread.hpp"

msg_t HeartbeatThread::main() {
  while(true) {
    palSetPad(GPIOE, GPIOE_LED3_RED);
    sleep(MS2ST(500));
    palClearPad(GPIOE, GPIOE_LED3_RED);
    sleep(MS2ST(500));
  }

  return 0;
}
