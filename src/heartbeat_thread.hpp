#ifndef HEARTBEAT_THREAD_HPP_
#define HEARTBEAT_THREAD_HPP_

#include "ch.hpp"
#include "hal.h"

#include "params/parameter_repository.hpp"

/**
 * A thread to blink an LED, showing the user that the control software is still
 * running (has not crashed).
 */
class HeartbeatThread : public chibios_rt::BaseStaticThread<0> {
public:
  static constexpr char const *PARAM_BLINK_FREQUENCY = "heartbeat_thread.blink_frequency";

  HeartbeatThread(ParameterRepository& params);
  msg_t main() override;

private:
    ParameterRepository& params;
};

#endif
