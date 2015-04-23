#include "filesystem/fs_writer_thread.hpp"

#include "chprintf.h"

FsWriterThread::FsWriterThread(SDCDriver& sdcd)
  : fs(sdcd) {
}

msg_t FsWriterThread::main() {
  while(true) {
    // Check if there is data in the buffer that has not yet been written.
    while(bottom != top) {
      fs.write(buffer[bottom++]);

      // Wrap if the end of the buffer is reached.
      if(bottom >= buffer.size()) {
        bottom = 0;
      }
    }

    // TODO(kyle): Just yield, or sleep?
    chThdSleepMicroseconds(10);   // TODO(yoos): yield in this thread makes things not work.
  }
}
