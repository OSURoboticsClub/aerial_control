template <typename M>
void Communicator::on(const M& m) {
  for(auto listener : listeners) {
    if(listener != nullptr) {
      listener->on(m);
    }
  }
}
