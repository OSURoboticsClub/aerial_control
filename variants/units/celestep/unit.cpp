#include "variant/unit.hpp"

Unit::Unit(Platform& platform, ParameterRepository& params, Communicator& communicator, Logger& logger)
  : data(platform, params, communicator, logger) {
}

VehicleSystem& Unit::getSystem() {
  return data.system;
}
