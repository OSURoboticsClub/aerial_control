#include "variant/unit.hpp"

Unit::Unit(Platform& platform, Communicator& communicator)
  : data(platform, communicator) {
}

void Unit::init() {
  data.system.init();
}

VehicleSystem& Unit::getSystem() {
  return data.system;
}
