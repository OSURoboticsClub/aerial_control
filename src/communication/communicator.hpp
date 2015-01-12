#ifndef COMMUNICATOR_HPP_
#define COMMUNICATOR_HPP_

#include <array>

#include "ch.hpp"
#include "hal.h"
#include "protocol/protocol.hpp"
#include "protocol/messages.hpp"
#include "protocol/encoder.hpp"
#include "protocol/decoder.hpp"

class Communicator {
public:
  Communicator(chibios_rt::BaseSequentialStreamInterface& stream);

  void submit(std::uint8_t b);

  template <typename M>
  void on(const M& m);

  template <typename M>
  void send(const M& message);

  chibios_rt::BaseSequentialStreamInterface& getStream();

private:
  template <std::size_t buffer_size>
  void dispatch(const protocol::decoded_message_t<buffer_size>& decoded);

  chibios_rt::BaseSequentialStreamInterface& stream;

  protocol::Encoder encoder;
  protocol::Decoder decoder;

  std::array<std::uint8_t, 255> encodeBuffer;
};

#include "communicator.tpp"
#include "communicator_handlers.tpp"

#endif
