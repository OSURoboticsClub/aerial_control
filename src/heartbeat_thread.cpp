#include "heartbeat_thread.hpp"

#include "params/parameter_repository.hpp"

#include "variant/indicator_platform.hpp"

HeartbeatThread::HeartbeatThread(ParameterRepository& params) : params(params) {
  params.def(PARAM_BLINK_FREQUENCY, 2);
}

msg_t HeartbeatThread::main() {
  auto& indicator = IndicatorPlatform::getInstance();

  while(true) {
    int sleepPeriod = 1000 / params.get(PARAM_BLINK_FREQUENCY); // Hz to ms

    indicator.set(IndicatorIntent::HEARTBEAT, true);
    sleep(MS2ST(sleepPeriod / 2));
    indicator.set(IndicatorIntent::HEARTBEAT, false);
    sleep(MS2ST(sleepPeriod / 2));
  }

  return 0;
}
