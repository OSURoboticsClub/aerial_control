#include "variant/unit.hpp"

Unit::Unit(Platform& platform, Communicator& communicator)
  : data(platform, communicator) {
}

VehicleSystem& Unit::getSystem() {
  return data.system;
}
