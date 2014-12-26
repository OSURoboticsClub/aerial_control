#include <array>

CommunicationThread::CommunicationThread(BaseChannel *channel)
  : channel(channel) {
}

msg_t CommunicationThread::main() {
  static std::array<std::uint8_t, 255> decodeBuffer;

  while(true) {
    std::size_t len = chnRead(channel, decodeBuffer.data(), decodeBuffer.size());

    for(std::size_t i = 0; i < len; i++) {
      protocol::decoded_message_t<255> decoded;
      if(decoder.process(decodeBuffer[i], &decoded)) {
        dispatch(decoded);
      }
    }
  }
}

template <std::size_t buffer_size>
void CommunicationThread::dispatch(const protocol::decoded_message_t<buffer_size>& decoded) {
  // TODO(kyle): Is there a way to do this without a giant switch?
  switch(decoded.id) {
    case protocol::message::heartbeat_message_t::ID: {
      auto message = reinterpret_cast<const protocol::message::heartbeat_message_t *>(&decoded.payload);
      on(message);
      break;
    }
  }
}

template <typename T>
void CommunicationThread::send(T m) {
  std::array<std::uint8_t, 255> encodeBuffer;
  std::uint16_t len = encoder.encode(m, &encodeBuffer);

  chnWrite(channel, encodeBuffer.data(), len);
}
