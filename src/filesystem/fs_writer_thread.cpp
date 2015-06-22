#include "filesystem/fs_writer_thread.hpp"

#include "protocol/messages.hpp"
#include "util/time.hpp"

#include "chprintf.h"

FsWriterThread::FsWriterThread(SDCDriver& sdcd, Communicator& communicator)
  : fs(sdcd),
    fsInfoMessageStream(communicator, 1) {
      rb_init(&buf, sizeof(_buf), _buf);
}

msg_t FsWriterThread::main() {
  BaseSequentialStream *chp = (BaseSequentialStream*)&SD1;
  chprintf(chp, "Filesystem thread online.\r\n");

  while (!fs.connect()) chThdSleepMilliseconds(10);
  while (!fs.mount())   chThdSleepMilliseconds(10);
  while (!fs.openNew()) chThdSleepMilliseconds(10);
  fsReady = true;

  while(true) {
    uint32_t count = buf.count;
    if (count > 3000) count = 3000 + 0.0625*(count-3000);
    rb_remove(&buf, count, writebuf);
    fs.write(writebuf, count);
    fs.sync();

    if (fsInfoMessageStream.ready()) {
      protocol::message::fs_info_message_t m;
      m.time = ST2MS(chibios_rt::System::getTime());
      fs.getFn(m.fname);
      m.fsize = fs.getFileSize();

      fsInfoMessageStream.publish(m);
    }

    yield();
  }

  // Should never get here
  fs.umount();
  fs.disconnect();
}
