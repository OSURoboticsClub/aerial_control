template <typename M>
void CommunicationThread::on(const M& m) {
  // empty for unhandled messages
}

template <>
void CommunicationThread::on(const protocol::message::heartbeat_message_t& m) {
  protocol::message::heartbeat_message_t message {
    .seq = m.seq
  };

  send(message);
}
