#include <ch.hpp>
#include <hal.h>

#include <array> // TODO(kyle):
#include <protocol.hpp>
#include <messages.hpp>
#include <encoder.hpp>
#include <decoder.hpp>

#include <hal_config.hpp>
#include <platform_config.hpp>

// Misc
#include <debugger.hpp>

static platform::HeartbeatThread heartbeatThread;
static Debugger debugger;

void communicate() {
  protocol::Encoder encoder;
  protocol::message::heartbeat_message_t message {
    .seq = 0x50
  };

  std::array<std::uint8_t, 3> buffer;
  encoder.encode(message, &buffer);

  chnWrite((BaseSequentialStream *) &SD1, buffer.data(), 3);
}

int main(void) {
  halInit();
  chibios_rt::System::init();

  // Start the background threads
  heartbeatThread.start(LOWPRIO);
  debugger.start(LOWPRIO);

  // Build and initialize the system
  platform::init();

  // Loop at a fixed rate forever
  // NOTE: If the deadline is ever missed then the loop will hang indefinitely.
  systime_t deadline = chTimeNow();
  while(true) {
    deadline += MS2ST(DT * 1000);

    platform::system.update();
    communicate();

    chibios_rt::BaseThread::sleepUntil(deadline);
  }

  return 0;
}
