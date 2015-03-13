#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include <array>

#include "ch.hpp"
#include "hal.h"
#include "filesystem/filesystem.hpp"
#include "protocol/protocol.hpp"

class Logger {
public:
  Logger(FileSystem& fs);
};

#endif
