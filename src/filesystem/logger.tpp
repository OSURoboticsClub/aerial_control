template <typename M>
void Logger::write(const M& message) {
  std::array<std::uint8_t, 255> encodeBuffer;
  std::uint16_t len = encoder.encode(message, &encodeBuffer);

  // Add to ringbuffer after filesystem has been initialized
  if (fsReady) {
    rb_add(&buf, len, encodeBuffer.data());
  }
}
