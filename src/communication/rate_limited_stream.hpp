#ifndef RATE_LIMITED_STREAM_HPP_
#define RATE_LIMITED_STREAM_HPP_

#include "ch.hpp"

#include "communication/communicator.hpp"

class RateLimitedStream {
public:
  RateLimitedStream(Communicator& communicator, int frequency);

  /**
   * Returns whether or not the rate limit delay has expired since the last
   * publish.
   */
  bool ready();

  /**
   * Send a message, regardless of the time since the last publish.
   */
  template <typename M>
  void publish(M& m);

private:
  Communicator& communicator;

  /**
   * Time between each message in milliseconds.
   */
  int period;

  /**
   * The time in system ticks of the last published message.
   */
  int lastPublish;
};

#include "rate_limited_stream.tpp"

#endif
