#include "filesystem/filesystem.hpp"

#include "chprintf.h"
#include "util/time.hpp"

#include <cstdio>
#include <cstring>

FileSystem::FileSystem(SDCDriver& sdcd)
  : sdcd(sdcd), fs_ready(false) {
}

bool FileSystem::connect(void) {
  BaseSequentialStream *chp = (BaseSequentialStream*)&SD4;
  chprintf(chp, "Trying to connect SDIO...");
  if (sdcConnect(&sdcd)) {
    chprintf(chp, " failed\r\n");
    return false;
  }
  chprintf(chp, " OK\r\n");

  return true;
}

bool FileSystem::disconnect(void) {
  BaseSequentialStream *chp = (BaseSequentialStream*)&SD4;
  chprintf(chp, "Disconnecting from SDIO...");
  if (sdcDisconnect(&sdcd)) {
    chprintf(chp, " failed\r\n");
    return false;
  }
  chprintf(chp, " OK\r\n");

  return true;
}

bool FileSystem::mount(void) {
  BaseSequentialStream *chp = (BaseSequentialStream*)&SD4;
  chprintf(chp, "Registering FS working area...");
  err = f_mount(0, &SDC_FS);
  if (err != FR_OK) {
    chprintf(chp, " failed\r\n");
    return false;
  }
  fs_ready = TRUE;
  chprintf(chp, " OK\r\n");

  chprintf(chp, "Mounting filesystem...");
  err = f_getfree("/", &clusters, &fsp);
  if (err != FR_OK) {
    chprintf(chp, " failed\r\n");
    return false;
  }
  chprintf(chp, " OK\r\n");
  chprintf(chp,
      "FS: %lu free clusters, %lu sectors per cluster, %lu bytes free\r\n",
      clusters, (uint32_t)SDC_FS.csize,
      clusters * (uint32_t)SDC_FS.csize * (uint32_t)MMCSD_BLOCK_SIZE);

  return true;
}

bool FileSystem::umount(void) {
  BaseSequentialStream *chp = (BaseSequentialStream*)&SD4;
  chprintf(chp, "Unmounting filesystem...");
  f_mount(0, NULL);
  if (err != FR_OK) {
    chprintf(chp, " failed\r\n");
    return false;
  }
  chprintf(chp, " OK\r\n");

  return true;
}

bool FileSystem::openNew(void) {
  BaseSequentialStream *chp = (BaseSequentialStream*)&SD4;
  chprintf(chp, "Generating filename...");
  uint16_t idx = 0;
  char fn[255];
  while (err != FR_NO_FILE) {
    chprintf(chp, ".");
    sprintf(fn, "celeste_%03d.log", idx++);
    err = f_stat(fn, &filinfo);

    if (idx == 1000) {
      chprintf(chp, " too many files\r\n");
      return false;
    }
  }
  chprintf(chp, " OK\r\n");

  return open(fn);
}

bool FileSystem::open(char *fn) {
  BaseSequentialStream *chp = (BaseSequentialStream*)&SD4;
  chprintf(chp, "Opening file %s...", fn);

  char fn_full[255];
  sprintf(fn_full, "0:%s", fn);
  err = f_open(&FileObject, fn_full, FA_WRITE | FA_OPEN_ALWAYS);
  if (err != FR_OK) {
    chprintf(chp, " failed\r\n");
    return false;
  }
  chprintf(chp, " OK\r\n");
  strcpy(fname, fn_full);

  return true;
}

bool FileSystem::close(void) {
  BaseSequentialStream *chp = (BaseSequentialStream*)&SD4;
  chprintf(chp, "Closing file...");
  err = f_close(&FileObject);
  if (err != FR_OK) {
    chprintf(chp, " failed\r\n");
    return false;
  }
  chprintf(chp, " OK\r\n");

  return true;
}

void FileSystem::read(uint8_t c) {
  if (sdcRead(&sdcd, 0, inbuf, SDC_BURST_SIZE)) {};
  // TODO(yoos);
}

void FileSystem::write(uint8_t *buf, uint16_t len) {
  BaseSequentialStream *chp = (BaseSequentialStream*)&SD4;
  uint8_t teststring[] = {"This is a test file\r\n"};
  err = f_write(&FileObject, buf, len, (unsigned int*)&bytes_written);
  if (err != FR_OK) {
    chprintf(chp, "write failed\r\n");
  }
  err = f_sync(&FileObject);   // TODO(yoos): This ensures data gets written, but bad for performance...
  if (err != FR_OK) {
    chprintf(chp, "sync failed\r\n");
  }
  if (bytes_written != len) {
    chprintf(chp, "write incomplete\r\n");
  }
  // TODO(yoos): On failure, reset logger to wait state
}

void FileSystem::getFn(char *buf) {
  std::strcpy(buf, fname);
}

uint32_t FileSystem::getFileSize(void) {
  return FileObject.fsize;
}

bool FileSystem::healthy(void) {
  uint32_t i=0;
  BaseSequentialStream *chp = (BaseSequentialStream*)&SD4;

  connect();

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
  uint8_t teststring[] = "This is a test file\n";

  mount();

  open("chtest.txt");

  chprintf(chp, "Writing data...");
  write(teststring, sizeof(teststring));
  chprintf(chp, " OK\r\n");

  close();

  chprintf(chp, "Checking file size \"chtest.txt\"...");
  err = f_stat("0:chtest.txt", &filinfo);
  if (err != FR_OK) {
    return false;
  }
  else{
    if (filinfo.fsize == sizeof(teststring))
      chprintf(chp, " OK\r\n");
    else
      return false;
  }

  chprintf(chp, "Check file content \"chtest.txt\"...");
  err = f_open(&FileObject, "0:chtest.txt", FA_READ | FA_OPEN_EXISTING);
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
      chprintf(chp, " OK\r\n");
    }
  }

  umount();
  disconnect();

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
