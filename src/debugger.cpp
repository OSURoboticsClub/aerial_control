#include <debugger.hpp>

#include <cstdarg>
#include <cstring>
#include <hal.h>
#include <memstreams.h>
#include <chprintf.h>

Debugger::Debugger() : chibios_rt::BaseStaticThread<256>() {
  // chPoolInit(&messagePool, sizeof(debug_message_t), NULL);
  //
  // for(int i = 0; i < 10; i++) {
  //   chPoolFree(&messagePool, &messages[i]);
  // }
  chMtxInit(&currentMessageLock);
}

msg_t Debugger::main() {
  while(true) {
    // TODO: chprintf is really slow at printing floats and causes us to
    // miss our deadline.
    // chprintf((BaseSequentialStream *) &SD1, "roll: %d pitch: %d yaw: %d\r\n",
    //     (int) estimate.roll, (int) estimate.pitch, (int) estimate.yaw);
    if(bufferFilled) {
      chMtxLock(&currentMessageLock);

      int i = 0;
      while(currentMessage[i] != '\0') {
        chSequentialStreamPut(&SD1, currentMessage[i++]);
      }

      bufferFilled = false;

      chMtxUnlock();
    }

    chsnprintf(currentMessage, 28, "test on usart1 %d\r\n", 1234567890);
    chnWriteTimeout((BaseChannel*)&SD1, (const uint8_t*)&currentMessage, 28, MS2ST(20));
    chsnprintf(currentMessage, 28, "test on usart3 %d\r\n", 1234567890);
    chnWriteTimeout((BaseChannel*)&SD3, (const uint8_t*)&currentMessage, 28, MS2ST(20));

    sleep(MS2ST(100));
  }

  return 0;
}

void Debugger::write(char *message) {
  // Wait for the buffer to become empty.
  while(bufferFilled) {
    sleep(US2ST(100));
  }

  chMtxLock(&currentMessageLock);
  strncpy(currentMessage, message, 255);
  chMtxUnlock();
}

void Debugger::printf(const char *fmt, ...) {
  // Wait for the buffer to become empty.
  while(bufferFilled) {
    sleep(US2ST(100));
  }

  chMtxLock(&currentMessageLock);

  va_list ap;
  MemoryStream ms;
  BaseSequentialStream *chp;

  /* Memory stream object to be used as a string writer.*/
  msObjectInit(&ms, (uint8_t *) currentMessage, 255, 0);

  /* Performing the print operation using the common code.*/
  chp = (BaseSequentialStream *)&ms;
  va_start(ap, fmt);
  chvprintf(chp, fmt, ap);
  va_end(ap);

  /* Final zero and size return.*/
  chSequentialStreamPut(chp, 0);

  bufferFilled = true;

  chMtxUnlock();
}


