#include <chprintf.h>

template <std::size_t size>
void FsWriterThread::append(std::array<std::uint8_t, size> ap, std::size_t len) {
  if (fsReady) {
    rb_add(&buf, len, ap.data());
  }
}
