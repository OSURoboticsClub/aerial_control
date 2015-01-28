#include "communication/rate_limited_stream.hpp"

RateLimitedStream::RateLimitedStream(Communicator& communicator, int frequency)
  : communicator(communicator), period(1000.0f / frequency),
    lastPublish(chibios_rt::System::getTime()) {
}

bool RateLimitedStream::ready() const {
  systime_t now = chibios_rt::System::getTime();
  return now >= lastPublish + MS2ST(period);
}

