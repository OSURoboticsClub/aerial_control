#include "filesystem/fs_writer_thread.hpp"

#include "chprintf.h"

FsWriterThread::FsWriterThread(SDCDriver& sdcd)
  : fs(sdcd) {
}

msg_t FsWriterThread::main() {
  BaseSequentialStream *chp = (BaseSequentialStream*)&SD4;
  chprintf(chp, "Filesystem thread online.\r\n");

  if (fs.connect()) {
    fs.mount();
  }

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
    chThdSleepMicroseconds(10000);   // TODO(yoos): yield in this thread makes things not work.
  }

  // Should never get here
  fs.umount();
  fs.disconnect();
}
