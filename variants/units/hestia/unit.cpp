#include "variant/unit.hpp"

Unit::Unit(Platform& platform, Communicator& communicator)
  : data(communicator) {
}

VehicleSystem& Unit::getSystem() {
  return data.system;
}
