#include "variant/unit.hpp"

Unit::Unit(Platform& platform) {
}

void Unit::init() {
  data.system.init();
}

VehicleSystem& Unit::getSystem() {
  return data.system;
}
