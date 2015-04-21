#include "filesystem/filesystem.hpp"

#include "chprintf.h"
#include "util/time.hpp"

FileSystem::FileSystem(SDCDriver& sdcd) {
  chprintf((BaseSequentialStream*)&SD4, "%d FS init\r\n", ST2MS(chibios_rt::System::getTime()));
}

void FileSystem::write(std::uint8_t c) {
  chprintf((BaseSequentialStream*)&SD4, "%d FS write\r\n", ST2MS(chibios_rt::System::getTime()));
}
