#ifndef READER_THREAD_HPP_
#define READER_THREAD_HPP_

#include <array>

#include "ch.hpp"

#include "communication/sink.hpp"

/**
 * Thread responsible for reading from an input stream (a blocking operation)
 * and relaying data to a Communicator. Its only purpose is to ensure the
 * control thread is not blocked by I/O.
 *
 * See `WriterThread`.
 */
class ReaderThread : public chibios_rt::BaseStaticThread<256> {
public:
  ReaderThread(chibios_rt::BaseSequentialStreamInterface& stream, Sink& sink);

  msg_t main() override;

private:
  chibios_rt::BaseSequentialStreamInterface& stream;
  Sink& sink;
};

#endif
