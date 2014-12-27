#ifndef COMMUNICATION_HPP_
#define COMMUNICATION_HPP_

#include <hal.h>

#include <protocol/protocol.hpp>
#include <protocol/messages.hpp>
#include <protocol/encoder.hpp>
#include <protocol/decoder.hpp>

class CommunicationThread : public chibios_rt::BaseStaticThread<256> {
public:
  CommunicationThread(BaseChannel *channel);

  msg_t main() override;

private:
  template <std::size_t buffer_size>
  void dispatch(const protocol::decoded_message_t<buffer_size>& decoded);

  template <typename M>
  void on(const M& message);

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
