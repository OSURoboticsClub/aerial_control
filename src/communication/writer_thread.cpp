#include "communication/writer_thread.hpp"

#include "hal.h"

WriterThread::WriterThread(chibios_rt::BaseSequentialStreamInterface& stream)
  : stream(stream) {
}

msg_t WriterThread::main() {
  while(true) {
    // Check if there is data in the buffer that has not yet been written.
    while(bottom != top) {
      stream.put(buffer[bottom++]);

      // Wrap if the end of the buffer is reached.
      if(bottom >= buffer.size()) {
        bottom = 0;
      }
    }

    // TODO(kyle): Just yield, or sleep?
    yield();
  }
}
