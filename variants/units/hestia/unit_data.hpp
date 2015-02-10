#ifndef UNIT_DATA_HPP_
#define UNIT_DATA_HPP_

#include "communication/communicator.hpp"
#include "system/dummy_vehicle_system.hpp"

struct UnitData {
  DummyVehicleSystem system;

  UnitData(Communicator& communicator)
    : system(communicator) {
  }
};

#endif
