#ifndef DEBUGGER_H_
#define DEBUGGER_H_

#include <ch.hpp>

class Debugger : public chibios_rt::BaseStaticThread<256> {
public:
  Debugger();

  virtual msg_t main();
};

#endif
