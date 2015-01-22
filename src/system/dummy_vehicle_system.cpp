#include "system/dummy_vehicle_system.hpp"

DummyVehicleSystem::DummyVehicleSystem(Communicator& communicator)
  : VehicleSystem(communicator) {
}

void DummyVehicleSystem::update() {
}
