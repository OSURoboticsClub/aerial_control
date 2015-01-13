#ifndef COMMUNICATOR_HPP_
#define COMMUNICATOR_HPP_

#include <array>

#include "ch.hpp"
#include "hal.h"
#include "protocol/protocol.hpp"
#include "protocol/messages.hpp"
#include "protocol/encoder.hpp"
#include "protocol/decoder.hpp"

#include "message_listener.hpp"

class Communicator {
public:
  Communicator(chibios_rt::BaseSequentialStreamInterface& stream);

  void submit(std::uint8_t b);

  template <typename M>
  void on(const M& m);

  template <typename M>
  void send(const M& message);

  void registerListener(MessageListener *listener);

  chibios_rt::BaseSequentialStreamInterface& getStream();

private:
  template <std::size_t buffer_size>
  void dispatch(const protocol::decoded_message_t<buffer_size>& decoded);

  chibios_rt::BaseSequentialStreamInterface& stream;

  protocol::Encoder encoder;
  protocol::Decoder decoder;

  // TODO(kyle): Fixed size array probably isn't good.
  std::array<MessageListener *, 10> listeners;
  std::size_t listenersPos;
};

#include "communicator.tpp"
#include "communicator_handlers.tpp"

#endif
