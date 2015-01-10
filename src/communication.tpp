CommunicationThread::CommunicationThread(Unit& unit, chibios_rt::BaseSequentialStreamInterface& stream)
  : unit(unit), stream(stream) {
}

msg_t CommunicationThread::main() {
  while(true) {
    std::uint8_t b = stream.get();

    protocol::decoded_message_t<255> decoded;
    if(decoder.process(b, &decoded)) {
      dispatch(decoded);
    }
  }
}

template <typename M, std::size_t buffer_size>
static const M& packet_cast(const protocol::decoded_message_t<buffer_size>& decoded) {
  return reinterpret_cast<const M&>(decoded.payload);
}

template <std::size_t buffer_size>
void CommunicationThread::dispatch(const protocol::decoded_message_t<buffer_size>& decoded) {
  // TODO(kyle): Is there a way to do this without a giant switch?
  switch(decoded.id) {
    case protocol::message::heartbeat_message_t::ID:
      on(packet_cast<protocol::message::heartbeat_message_t>(decoded));
      break;
    case protocol::message::log_message_t::ID:
      on(packet_cast<protocol::message::log_message_t>(decoded));
      break;
    case protocol::message::attitude_message_t::ID:
      on(packet_cast<protocol::message::attitude_message_t>(decoded));
      break;
    case protocol::message::set_control_mode_message_t::ID:
      on(packet_cast<protocol::message::set_control_mode_message_t>(decoded));
      break;
  }
}

template <typename M>
void CommunicationThread::send(const M& message) {
  std::uint16_t len = encoder.encode(message, &encodeBuffer);

  stream.write(encodeBuffer.data(), len);
}
