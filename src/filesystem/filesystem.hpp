#ifndef FILESYSTEM_HPP_
#define FILESYSTEM_HPP_

#include "ch.hpp"
#include "hal.h"

#if HAL_USE_SDC==TRUE
#define USE_FILESYSTEM TRUE

#include "ff.h"

#include <cstdint>
#include <cstring>

#define SDC_BURST_SIZE 8   // How many sectors to read at once

static uint8_t outbuf[MMCSD_BLOCK_SIZE * SDC_BURST_SIZE + 1];
static uint8_t  inbuf[MMCSD_BLOCK_SIZE * SDC_BURST_SIZE + 1];

class FileSystem {
public:
  FileSystem(SDCDriver& sdcd);

  /**
   * Connect to SDIO
   */
  bool connect(void);

  /**
   * Disconnect from SDIO
   */
  bool disconnect(void);

  /**
   * Mount filesystem
   */
  bool mount(void);

  /**
   * Unmount filesystem
   */
  bool umount(void);

  /**
   * Open new file
   *
   * This will generate a filename as to avoid overwriting existing log files.
   */
  bool openNew(void);

  /**
   * Open file by name
   */
  bool open(char *fn);

  /**
   * Close current file
   */
  bool close(void);

  /**
   * Read from filesystem.
   */
  void read(uint8_t c);

  /**
   * Write to filesystem.
   */
  void write(uint8_t *c, uint16_t len);

  /**
   * Sync filesystem.
   */
  void sync(void);

  /**
   * Get short filename
   */
  void getFn(char *buf);

  /**
   * Get file size
   */
  uint32_t getFileSize(void);

  /**
   * Health check.
   */
  bool healthy(void);

private:
  SDCDriver& sdcd;

  void fillbuffer(uint8_t pattern, uint8_t *b);
  void fillbuffers(uint8_t pattern);

  char fname[32];   // File name. TODO(yoos): I thought filinfo.lfname would give me this, but there's nothing there...
  FATFS SDC_FS;   // FS object
  bool_t fs_ready;   // FS mounted and ready
  FRESULT err;
  uint32_t clusters;
  FATFS *fsp;
  FIL FileObject;
  uint32_t bytes_written;
  uint32_t bytes_read;
  FILINFO filinfo;
};

#endif // HAL_USE_SDC==TRUE

#endif
