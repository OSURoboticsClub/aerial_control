#ifndef COMMUNICATION_HPP_
#define COMMUNICATION_HPP_

#include <hal.h>

#include <protocol/protocol.hpp>
#include <protocol/messages.hpp>
#include <protocol/encoder.hpp>
#include <protocol/decoder.hpp>

template <std::size_t buffer_size>
class CommunicationThread : public chibios_rt::BaseStaticThread<256> {
public:
  CommunicationThread(BaseChannel *channel);

  msg_t main() override;

private:
  void dispatch(const protocol::decoded_message_t<buffer_size>& decoded);

  template <typename M>
  void on(const M& message);

  // TODO: This is overridden for now because we can't specialize on M without
  // specializing on buffer_size as well.
  void on(const protocol::message::heartbeat_message_t& m);

  template <typename M>
  void send(const M& message);

  BaseChannel *channel;

  protocol::Encoder encoder;
  protocol::Decoder decoder;
};

// Handlers included before main implementation to avoid template use before
// explicit specialization (C++ n3376 14.7.3/6).
#include "communication_handlers.tpp"

#include "communication.tpp"

#endif
