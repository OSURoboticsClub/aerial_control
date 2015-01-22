#ifndef CONTROL_THREAD_HPP_
#define CONTROL_THREAD_HPP_

#include "ch.hpp"

class ControlThread : public chibios_rt::BaseStaticThread<2048> {
  msg_t main() override;
};

#endif
