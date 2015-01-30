#ifndef HEARTBEAT_THREAD_HPP_
#define HEARTBEAT_THREAD_HPP_

#include "ch.hpp"
#include "hal.h"

/**
 * A thread to blink an LED, showing the user that the control software is still
 * running (has not crashed).
 */
class HeartbeatThread : public chibios_rt::BaseStaticThread<0> {
public:
  msg_t main() override;
};

#endif
