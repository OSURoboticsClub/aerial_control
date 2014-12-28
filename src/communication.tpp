#include <array>

template <std::size_t buffer_size>
CommunicationThread<buffer_size>::CommunicationThread(chibios_rt::BaseSequentialStreamInterface *channel)
  : channel(channel) {
}

template <std::size_t buffer_size>
msg_t CommunicationThread<buffer_size>::main() {
  static std::array<std::uint8_t, 255> decodeBuffer;

  while(true) {
    std::size_t len = channel->read(decodeBuffer.data(), decodeBuffer.size());

    for(std::size_t i = 0; i < len; i++) {
      protocol::decoded_message_t<255> decoded;
      if(decoder.process(decodeBuffer[i], &decoded)) {
        dispatch(decoded);
      }
    }
  }
}

template <std::size_t buffer_size>
void CommunicationThread<buffer_size>::dispatch(const protocol::decoded_message_t<buffer_size>& decoded) {
  // TODO(kyle): Is there a way to do this without a giant switch?
  switch(decoded.id) {
    case protocol::message::heartbeat_message_t::ID: {
      auto message = reinterpret_cast<const protocol::message::heartbeat_message_t *>(&decoded.payload);
      on(message);
      break;
    }
  }
}

template <std::size_t buffer_size> template <typename M>
void CommunicationThread<buffer_size>::send(const M& message) {
  std::array<std::uint8_t, 255> encodeBuffer;
  std::uint16_t len = encoder.encode(message, &encodeBuffer);

  channel->write(encodeBuffer.data(), len);
}
