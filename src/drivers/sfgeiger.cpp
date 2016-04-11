#include "drivers/sfgeiger.hpp"

#include <cstdlib>
#include <cstring>

#include "chprintf.h"

void SFGeiger::init() {
}

GeigerReading SFGeiger::readGeiger() {
  // Read all available bytes.
  std::uint8_t blips = read(0);

  return GeigerReading {
    .blips = blips
  };
}

bool SFGeiger::healthy() {
  return true;
}
