#include "heartbeat_thread.hpp"

#include "variant/indicator_platform.hpp"

msg_t HeartbeatThread::main() {
  auto& indicator = IndicatorPlatform::getInstance();
  while(true) {
    indicator.set(IndicatorIntent::HEARTBEAT, true);
    sleep(MS2ST(500));
    indicator.set(IndicatorIntent::HEARTBEAT, false);
    sleep(MS2ST(500));
  }

  return 0;
}
