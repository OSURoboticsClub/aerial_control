template <std::size_t buffer_size> template <typename M>
void CommunicationThread<buffer_size>::on(const M& m) {
  // empty for unhandled messages
}

template <std::size_t buffer_size>
void CommunicationThread<buffer_size>::on(const protocol::message::heartbeat_message_t& m) {
  protocol::message::heartbeat_message_t message {
    .seq = m.seq
  };

  send(message);
}
