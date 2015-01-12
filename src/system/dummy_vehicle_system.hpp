#ifndef DUMMY_VEHICLE_SYSTEM_HPP_
#define DUMMY_VEHICLE_SYSTEM_HPP_

#include "communication/communicator.hpp"
#include "system/vehicle_system.hpp"

class DummyVehicleSystem : public VehicleSystem {
public:
  DummyVehicleSystem(Communicator& communicator);

  void init() override;
  void update() override;
};

#endif
