#ifndef FS_WRITER_THREAD_HPP_
#define FS_WRITER_THREAD_HPP_

#include <array>

#include "ch.hpp"
#include "hal.h"

#include "filesystem/filesystem.hpp"

/**
 * Thread to write to storage media without blocking control thread.
 */
class FsWriterThread : public chibios_rt::BaseStaticThread<2048> {
public:
  FsWriterThread(SDCDriver& sdcd);

  msg_t main(void) override;

  /**
   * Append data to internal write buffer.
   */
  template <std::size_t size>
  void append(std::array<std::uint8_t, size> ap, std::size_t len);

private:
  FileSystem fs;

  std::array<std::uint8_t, 20000> buffer;
  std::size_t bottom;
  std::size_t top;
};

#include "filesystem/fs_writer_thread.tpp"

#endif
