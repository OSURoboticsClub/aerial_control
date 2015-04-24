#include "filesystem/fs_writer_thread.hpp"

#include "protocol/messages.hpp"
#include "util/time.hpp"

#include "chprintf.h"

FsWriterThread::FsWriterThread(SDCDriver& sdcd, Communicator& communicator)
  : fs(sdcd),
    fsInfoMessageStream(communicator, 1) {
}

msg_t FsWriterThread::main() {
  BaseSequentialStream *chp = (BaseSequentialStream*)&SD4;
  chprintf(chp, "Filesystem thread online.\r\n");

  while (!fs.connect()) chThdSleepMilliseconds(10);
  while (!fs.mount())   chThdSleepMilliseconds(10);
  while (!fs.openNew()) chThdSleepMilliseconds(10);

  while(true) {
    // Check if there is data in the buffer that has not yet been written.
    if(bottom != top) {
      size_t numbytes = (top > bottom) ? top-bottom : buffer.size()-bottom;
      fs.write(&buffer[bottom], numbytes);
      bottom += numbytes;

      // Wrap if the end of the buffer is reached.
      if(bottom >= buffer.size()) {
        bottom = 0;
      }
    }

    if (fsInfoMessageStream.ready()) {
      protocol::message::fs_info_message_t m;
      m.time = ST2MS(chibios_rt::System::getTime());
      fs.getFn(m.fname);
      m.fsize = fs.getFileSize();

      fsInfoMessageStream.publish(m);
    }

    // TODO(kyle): Just yield, or sleep?
    yield();
  }

  // Should never get here
  fs.umount();
  fs.disconnect();
}
