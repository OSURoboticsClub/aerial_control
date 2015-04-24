#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include <array>

#include "ch.hpp"
#include "hal.h"
#include "protocol/protocol.hpp"

#include "communication/communicator.hpp"
#include "filesystem/filesystem.hpp"
#include "filesystem/fs_writer_thread.hpp"

class Logger {
public:
  Logger(SDCDriver& sdcd, Communicator& communicator);

  /**
   * Start writer threads.
   */
  void start(void);

  /**
   * Check if ready.
   */
  bool ready(void);

  /**
   * Write to filesystem.
   */
  template <typename M>
  void write(const M& message);

private:
  FsWriterThread writer;

  protocol::Encoder encoder;
};

#include "logger.tpp"

#endif
