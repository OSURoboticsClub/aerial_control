template <typename M>
void Logger::write(const M& message) {
  std::array<std::uint8_t, 255> encodeBuffer;
  std::uint16_t len = encoder.encode(message, &encodeBuffer);

  // Offload work onto the writer thread
  writer.append(encodeBuffer, len);
}
