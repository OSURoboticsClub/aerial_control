#include "filesystem/filesystem.hpp"

#include "chprintf.h"
#include "util/time.hpp"

#include <cstring>

FileSystem::FileSystem(SDCDriver& sdcd)
  : sdcd(sdcd) {
}

void FileSystem::write(std::uint8_t c) {
  //TODO(yoos);
}

bool FileSystem::healthy(void) {
  uint32_t i=0;
  static FATFS SDC_FS;   // FS object
  static bool_t fs_ready = FALSE;   // FS mounted and ready
  BaseSequentialStream *chp = (BaseSequentialStream*)&SD4;
  chprintf(chp, "Trying to connect SDIO... ");

  if (sdcConnect(&sdcd))
    return false;
  chprintf(chp, "OK\r\n");
  chprintf(chp, "*** Card CSD content: ");
  chprintf(chp, "%X %X %X %X \r\n", (&sdcd)->csd[3], (&sdcd)->csd[2], (&sdcd)->csd[1], (&sdcd)->csd[0]);


  chprintf(chp, "Single aligned read...");
  chThdSleepMilliseconds(100);
  if (sdcRead(&sdcd, 0, inbuf, 1))
    return false;
  chprintf(chp, " OK\r\n");
  chThdSleepMilliseconds(100);


  chprintf(chp, "Single unaligned read...");
  chThdSleepMilliseconds(100);
  if (sdcRead(&sdcd, 0, inbuf + 1, 1))
    return false;
  if (sdcRead(&sdcd, 0, inbuf + 2, 1))
    return false;
  if (sdcRead(&sdcd, 0, inbuf + 3, 1))
    return false;
  chprintf(chp, " OK\r\n");
  chThdSleepMilliseconds(100);


  chprintf(chp, "Multiple aligned reads...");
  chThdSleepMilliseconds(100);
  fillbuffers(0x55);
  /* fill reference buffer from SD card */
  if (sdcRead(&sdcd, 0, inbuf, SDC_BURST_SIZE))
    return false;
  for (i=0; i<1000; i++){
    if (sdcRead(&sdcd, 0, outbuf, SDC_BURST_SIZE))
      return false;
    if (memcmp(inbuf, outbuf, SDC_BURST_SIZE * MMCSD_BLOCK_SIZE) != 0)
      return false;
  }
  chprintf(chp, " OK\r\n");
  chThdSleepMilliseconds(100);


  chprintf(chp, "Multiple unaligned reads...");
  chThdSleepMilliseconds(100);
  fillbuffers(0x55);
  /* fill reference buffer from SD card */
  if (sdcRead(&sdcd, 0, inbuf + 1, SDC_BURST_SIZE))
    return false;
  for (i=0; i<1000; i++){
    if (sdcRead(&sdcd, 0, outbuf + 1, SDC_BURST_SIZE))
      return false;
    if (memcmp(inbuf, outbuf, SDC_BURST_SIZE * MMCSD_BLOCK_SIZE) != 0)
      return false;
  }
  chprintf(chp, " OK\r\n");
  chThdSleepMilliseconds(100);


  /**
   * Now perform some FS tests.
   */
  FRESULT err;
  uint32_t clusters;
  FATFS *fsp;
  FIL FileObject;
  uint32_t bytes_written;
  uint32_t bytes_read;
  FILINFO filinfo;
  uint8_t teststring[] = {"This is test file\r\n"};

  chprintf(chp, "Register working area for filesystem... ");
  chThdSleepMilliseconds(100);
  err = f_mount(0, &SDC_FS);
  if (err != FR_OK){
    return false;
  }
  else{
    fs_ready = TRUE;
    chprintf(chp, "OK\r\n");
  }

  chprintf(chp, "Mount filesystem... ");
  chThdSleepMilliseconds(100);
  err = f_getfree("/", &clusters, &fsp);
  if (err != FR_OK) {
    return false;
  }
  chprintf(chp, "OK\r\n");
  chprintf(chp,
      "FS: %lu free clusters, %lu sectors per cluster, %lu bytes free\r\n",
      clusters, (uint32_t)SDC_FS.csize,
      clusters * (uint32_t)SDC_FS.csize * (uint32_t)MMCSD_BLOCK_SIZE);


  chprintf(chp, "Create file \"chtest.txt\"... ");
  chThdSleepMilliseconds(100);
  err = f_open(&FileObject, "0:chtest.txt", FA_WRITE | FA_OPEN_ALWAYS);
  if (err != FR_OK) {
    return false;
  }
  chprintf(chp, "OK\r\n");
  chprintf(chp, "Write some data in it... ");
  chThdSleepMilliseconds(100);
  err = f_write(&FileObject, teststring, sizeof(teststring), (unsigned int*)&bytes_written);
  if (err != FR_OK) {
    return false;
  }
  else
    chprintf(chp, "OK\r\n");

  chprintf(chp, "Close file \"chtest.txt\"... ");
  err = f_close(&FileObject);
  if (err != FR_OK) {
    return false;
  }
  else
    chprintf(chp, "OK\r\n");

  chprintf(chp, "Check file size \"chtest.txt\"... ");
  err = f_stat("0:chtest.txt", &filinfo);
  chThdSleepMilliseconds(100);
  if (err != FR_OK) {
    return false;
  }
  else{
    if (filinfo.fsize == sizeof(teststring))
      chprintf(chp, "OK\r\n");
    else
      return false;
  }

  chprintf(chp, "Check file content \"chtest.txt\"... ");
  err = f_open(&FileObject, "0:chtest.txt", FA_READ | FA_OPEN_EXISTING);
  chThdSleepMilliseconds(100);
  if (err != FR_OK) {
    return false;
  }
  uint8_t buf[sizeof(teststring)];
  err = f_read(&FileObject, buf, sizeof(teststring), (unsigned int*)&bytes_read);
  if (err != FR_OK) {
    return false;
  }
  else{
    if (memcmp(teststring, buf, sizeof(teststring)) != 0){
      return false;
    }
    else{
      chprintf(chp, "OK\r\n");
    }
  }

  chprintf(chp, "Umount filesystem... ");
  f_mount(0, NULL);
  chprintf(chp, "OK\r\n");

  chprintf(chp, "Disconnecting from SDIO...");
  chThdSleepMilliseconds(100);
  if (sdcDisconnect(&sdcd))
    return false;
  chprintf(chp, " OK\r\n");
  chprintf(chp, "------------------------------------------------------\r\n");
  chprintf(chp, "All tests passed successfully.\r\n");

  return true;
}

void FileSystem::fillbuffer(uint8_t pattern, uint8_t *b) {
  uint32_t i = 0;
  for (i=0; i < MMCSD_BLOCK_SIZE * SDC_BURST_SIZE; i++)
    b[i] = pattern;
}

void FileSystem::fillbuffers(uint8_t pattern) {
  fillbuffer(pattern, inbuf);
  fillbuffer(pattern, outbuf);
}
