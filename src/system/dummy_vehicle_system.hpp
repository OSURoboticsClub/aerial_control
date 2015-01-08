#ifndef DUMMY_VEHICLE_SYSTEM_HPP_
#define DUMMY_VEHICLE_SYSTEM_HPP_

#include "system/vehicle_system.hpp"

class DummyVehicleSystem : public VehicleSystem {
public:
  void init() override;
  void update() override;
};

#endif
