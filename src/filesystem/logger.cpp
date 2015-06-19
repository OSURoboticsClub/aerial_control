#include "filesystem/logger.hpp"

Logger::Logger(SDCDriver& sdcd, Communicator& communicator)
  : writer(sdcd, communicator) {
}

void Logger::start(void) {
  // Start writer at higher priority than communicator threads.
  // TODO(yoos): Rethink this, as in the rocket, we really want to log data
  // before we transmit to ground, but this is not the case in radio-controlled
  // vehicles. Maybe priorities should be user-configurable.
  writer.start(HIGHPRIO-1);
}

bool Logger::ready(void) {
  // TODO(yoos)
  return true;
}
