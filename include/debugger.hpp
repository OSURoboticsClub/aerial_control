#ifndef DEBUGGER_HPP_
#define DEBUGGER_HPP_

#include <ch.hpp>

class Debugger : public chibios_rt::BaseStaticThread<256> {
public:
  Debugger();

  virtual msg_t main();
};

#endif
