#ifndef RATE_LIMITED_STREAM_HPP_
#define RATE_LIMITED_STREAM_HPP_

#include "ch.hpp"

#include "communication/communicator.hpp"

/**
 * A utility class to prevent a message generator from saturating an output
 * channel. Provides an easy way to limit message sending to a specified
 * maximum frequency.
 */
class RateLimitedStream {
public:
  RateLimitedStream(Communicator& communicator, int frequency);

  /**
   * Returns whether or not the rate limit delay has expired since the last
   * publish.
   */
  bool ready() const;

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
  systime_t period;

  /**
   * The time in system ticks of the last published message.
   */
  systime_t lastPublish;
};

#include "rate_limited_stream.tpp"

#endif
