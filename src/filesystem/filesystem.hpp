#ifndef FILESYSTEM_HPP_
#define FILESYSTEM_HPP_

#include "ch.hpp"
#include "hal.h"

#include "ff.h"

#include <cstdint>

#define SDC_BURST_SIZE 8   // How many sectors to read at once

static uint8_t outbuf[MMCSD_BLOCK_SIZE * SDC_BURST_SIZE + 1];
static uint8_t  inbuf[MMCSD_BLOCK_SIZE * SDC_BURST_SIZE + 1];

class FileSystem {
public:
  FileSystem(SDCDriver& sdcd);

  /**
   * Write to filesystem.
   */
  void write(std::uint8_t c);

  /**
   * Health check.
   */
  bool healthy(void);

private:
  SDCDriver& sdcd;

  void fillbuffer(uint8_t pattern, uint8_t *b);
  void fillbuffers(uint8_t pattern);
};

#endif
