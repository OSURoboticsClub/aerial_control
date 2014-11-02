#ifndef DEBUGGER_HPP_
#define DEBUGGER_HPP_

#include <ch.hpp>

// struct debug_message_t {
//   char message[255];
// };
//
// MemoryPool messagePool;
// debug_message_t messages[10];

static char currentMessage[255];
static Mutex currentMessageLock;

static bool bufferFilled;

class Debugger : public chibios_rt::BaseStaticThread<256> {
public:
  Debugger();

  virtual msg_t main();

  void write(char *message);
  void printf(const char *fmt, ...);
};

#endif
