#include "communication/reader_thread.hpp"

ReaderThread::ReaderThread(chibios_rt::BaseSequentialStreamInterface& stream, Sink& sink)
  : stream(stream),
    sink(sink) {
}

msg_t ReaderThread::main() {
  while(true) {
    sink.submit(stream.get());
  }

  return 0;
}
