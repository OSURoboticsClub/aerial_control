template <std::size_t size>
void WriterThread::append(std::array<std::uint8_t, size> ap, std::size_t len) {
  for(std::size_t i = 0; i < len; i++) {
    buffer[top++] = ap[i];

    if(top >= buffer.size()) {
      top = 0;
    }
  }
}
