#include "variant/unit.hpp"

Unit::Unit(Platform& platform, Communicator& communicator, Logger& logger)
  : data(platform, communicator, logger) {
}

VehicleSystem& Unit::getSystem() {
  return data.system;
}
