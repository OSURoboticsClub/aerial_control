#include <ch.hpp>
#include <hal.h>

#include <array> // TODO(kyle):
#include <protocol/protocol.hpp>
#include <protocol/messages.hpp>
#include <protocol/encoder.hpp>
#include <protocol/decoder.hpp>

#include <hal_config.hpp>
#include <platform_config.hpp>

// Misc
#include <debugger.hpp>

static platform::HeartbeatThread heartbeatThread;
static Debugger debugger;

void communicate() {
  protocol::Encoder encoder;
  std::array<std::uint8_t, 255> buffer;

  protocol::message::attitude_message_t message {
    .roll = 0.0f,
    .pitch = 0.0f,
    .yaw = 0.0f,
  };

  // protocol::message::heartbeat_message_t message {
  //   .seq = 0x00
  // };

  std::uint16_t len = encoder.encode(message, &buffer);

  sdWrite(&SD1, buffer.data(), len);
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
