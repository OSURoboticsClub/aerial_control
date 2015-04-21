#ifndef FILESYSTEM_HPP_
#define FILESYSTEM_HPP_

#include "ch.hpp"
#include "hal.h"

#include <cstdint>

// TODO(yoos): Find and inherit some FAT filesystem
class FileSystem {
public:
  FileSystem(SDCDriver& sdcd);

  /**
   * Write to filesystem.
   */
  void write(std::uint8_t c);
};

#endif
