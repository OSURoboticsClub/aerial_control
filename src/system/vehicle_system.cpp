#include "system/vehicle_system.hpp"

#include "variant/indicator_platform.hpp"

VehicleSystem::VehicleSystem(Communicator& communicator)
  : communicator(communicator) {
}

bool VehicleSystem::isArmed() const {
  return armed;
}

void VehicleSystem::setArmed(bool armed_) {
  armed = armed_;

  auto& indicator = IndicatorPlatform::getInstance();
  indicator.set(IndicatorIntent::VEHICLE_ARMED, armed);
}
