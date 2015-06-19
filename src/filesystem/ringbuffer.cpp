#include <filesystem/ringbuffer.hpp>

#include "util/time.hpp"
#include "ch.hpp"
#include "hal.h"
#include <chprintf.h>

static uint16_t overflowCount = 0;

void rb_init(rb_t *buf, std::size_t size, uint8_t *elems)
{
  buf->size = size;
  buf->count = 0;
  buf->head = 0;
  buf->elems = elems;
}

std::size_t rb_add(rb_t *buf, std::size_t num_bytes, uint8_t *input)
{
  BaseSequentialStream *chp = (BaseSequentialStream*)&SD4;
  /* Check if buffer is too full. */
  if (buf->size - buf->count < num_bytes) {
    overflowCount++;
    chprintf(chp, "RB %d %d %d %d ###\r\n", ST2MS(chibios_rt::System::getTime()), overflowCount, num_bytes, buf->count);

    // Drop a few bytes
    int dropCount = 10000;
    buf->head = (buf->head + dropCount) % buf->size;
    buf->count -= dropCount;
  }

  /* Copy data. */
  static uint16_t i;
  static uint32_t tail;
  tail = (buf->head + buf->count) % buf->size;
  for (i=0; i<num_bytes; i++) {
    buf->elems[tail++] = input[i];
    if (tail == buf->size) tail = 0;
  }

  buf->count += num_bytes;

  return num_bytes;
}

std::size_t rb_remove(rb_t *buf, std::size_t num_bytes, uint8_t *output)
{
  BaseSequentialStream *chp = (BaseSequentialStream*)&SD4;
  /* Check if buffer does not contain enough data. */
  if (buf->count < num_bytes) {
    num_bytes = buf->count;
  }

  /* Copy data. */
  static uint16_t i;
  for (i=0; i<num_bytes; i++) {
    output[i] = buf->elems[buf->head++];
    if (buf->head == buf->size) buf->head = 0;
  }

  // Crudely calculate average throughput
  static uint32_t avgBps = 0;
  static systime_t lastTime = 0;
  avgBps = 0.01*num_bytes*1000/(ST2MS(chibios_rt::System::getTime()-lastTime)) + 0.99*avgBps;
  lastTime = chibios_rt::System::getTime();
  chprintf(chp, "RB<%d> %d %d %d\r\n", ST2MS(chibios_rt::System::getTime()), overflowCount, avgBps, buf->count);


  buf->count -= num_bytes;

  return num_bytes;
}

std::size_t rb_peek(rb_t *buf, std::size_t num_bytes, uint8_t *output)
{
  /* Check if buffer does not contain enough data. */
  if (buf->count < num_bytes) {
    num_bytes = buf->count;
  }

  /* Copy data. */
  static uint16_t i;
  uint32_t idx = buf->head;
  for (i=0; i<num_bytes; i++) {
    output[i] = buf->elems[idx++];
    if (idx == buf->size) idx = 0;
  }

  return num_bytes;
}

