#include "system/vehicle_system.hpp"

VehicleSystem::VehicleSystem(Communicator& communicator)
  : communicator(communicator) {
}

bool VehicleSystem::isArmed() const {
  return armed;
}

void VehicleSystem::setArmed(bool armed_) {
  armed = armed_;
}
