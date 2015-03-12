#ifndef WRITER_THREAD_HPP_
#define WRITER_THREAD_HPP_

#include <array>

#include "ch.hpp"

/**
 * Thread responsible for writing to an output stream to avoid performing
 * blocking operations on the control thread.
 *
 * See `ReaderThread`.
 */
class WriterThread : public chibios_rt::BaseStaticThread<256> {
public:
  WriterThread(chibios_rt::BaseSequentialStreamInterface& stream);

  msg_t main() override;

  /**
   * Append data to the internal buffer waiting to be written.
   *
   * NOTE: If the internal buffer overflows then bad things could happen.
   */
  template <std::size_t size>
  void append(std::array<std::uint8_t, size> ap, std::size_t len);

private:
  chibios_rt::BaseSequentialStreamInterface& stream;

  std::array<std::uint8_t, 255> buffer;
  std::size_t bottom;
  std::size_t top;
};

#include "communication/writer_thread.tpp"

#endif
