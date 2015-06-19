#ifndef FS_WRITER_THREAD_HPP_
#define FS_WRITER_THREAD_HPP_

#include <array>

#include "ch.hpp"
#include "hal.h"

#include "communication/communicator.hpp"
#include "communication/rate_limited_stream.hpp"
#include "filesystem/filesystem.hpp"
#include "filesystem/ringbuffer.hpp"

/**
 * Thread to write to storage media without blocking control thread.
 */
class FsWriterThread : public chibios_rt::BaseStaticThread<1024> {
public:
  FsWriterThread(SDCDriver& sdcd, Communicator& communicator);

  msg_t main(void) override;

  /**
   * Append data to internal write buffer.
   */
  template <std::size_t size>
  void append(std::array<std::uint8_t, size> ap, std::size_t len);

private:
  FileSystem fs;
  bool fsReady;
  RateLimitedStream fsInfoMessageStream;

  rb_t buf;
  std::uint8_t _buf[83000];
  std::uint8_t writebuf[8000];
};

#include "filesystem/fs_writer_thread.tpp"

#endif
