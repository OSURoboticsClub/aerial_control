#include "filesystem/fs_writer_thread.hpp"

#include "chprintf.h"

FsWriterThread::FsWriterThread(SDCDriver& sdcd)
  : fs(sdcd) {
}

msg_t FsWriterThread::main() {
  BaseSequentialStream *chp = (BaseSequentialStream*)&SD4;
  chprintf(chp, "Filesystem thread online.\r\n");

  while (!fs.connect()) chThdSleepMilliseconds(10);
  while (!fs.mount())   chThdSleepMicroseconds(10);
  while (!fs.openNew()) chThdSleepMilliseconds(10);

  while(true) {
    // Check if there is data in the buffer that has not yet been written.
    while(bottom != top) {
      size_t numbytes = (top > bottom) ? top-bottom : buffer.size()-bottom;
      fs.write(&buffer[bottom], numbytes);
      bottom += numbytes;

      // Wrap if the end of the buffer is reached.
      if(bottom >= buffer.size()) {
        bottom = 0;
      }
    }

    // TODO(kyle): Just yield, or sleep?
    yield();
  }

  // Should never get here
  fs.umount();
  fs.disconnect();
}
