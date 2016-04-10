#include "filesystem/logger.hpp"

#if USE_FILESYSTEM==TRUE

#include "protocol/messages.hpp"
#include "util/time.hpp"
#include "variant/sdc_platform.hpp"
#include "variant/usart_platform.hpp"

#include "chprintf.h"

Logger::Logger(Platform& platform)
  : fs(platform.get<SDCPlatform>().getSDCDriver()),
    platform(platform) {
  rb_init(&buf, sizeof(_buf), _buf);
}

msg_t Logger::main() {
  BaseSequentialStream *chp = (BaseSequentialStream*)&platform.get<USARTPlatform>().getPrimaryStream();
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

    //if (fsInfoMessageStream.ready()) {
    //  protocol::message::fs_info_message_t m;
    //  m.time = ST2MS(chibios_rt::System::getTime());
    //  fs.getFn(m.fname);
    //  m.fsize = fs.getFileSize();

    //  fsInfoMessageStream.publish(m);
    //}

    yield();
  }

  // Should never get here
  fs.umount();
  fs.disconnect();
}

#endif
