#ifndef CONTROL_THREAD_HPP_
#define CONTROL_THREAD_HPP_

#include "ch.hpp"

/**
 * The main control thread. This thread is responsible for spinning off and
 * managing the heartbeat and communication subthreads, as well as running the
 * fixed interval control loop.
 */
class ControlThread : public chibios_rt::BaseStaticThread<4096> {
public:
  msg_t main() override;
};

#endif
