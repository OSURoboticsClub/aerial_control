#ifndef HEARTBEAT_THREAD_HPP_
#define HEARTBEAT_THREAD_HPP_

#include "ch.hpp"
#include "hal.h"

class HeartbeatThread : public chibios_rt::BaseStaticThread<0> {
public:
  msg_t main() override;
};

#endif
