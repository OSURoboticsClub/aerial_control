#include "system/vehicle_system.hpp"

bool VehicleSystem::isArmed() {
  return armed;
}

void VehicleSystem::setArmed(bool armed_) {
  armed = armed_;
}
