#ifndef UNIT_DATA_HPP_
#define UNIT_DATA_HPP_

#include "system/dummy_vehicle_system.hpp"

struct unit_data_t {
  DummyVehicleSystem system;

  unit_data_t(Communicator& communicator)
    : system(communicator) {
  }
};

#endif
