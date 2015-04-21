#ifndef UNIT_HPP_
#define UNIT_HPP_

#include "unit_data.hpp"

#include "communication/communicator.hpp"
#include "filesystem/logger.hpp"
#include "system/vehicle_system.hpp"
#include "variant/platform.hpp"

class Unit {
public:
  Unit(Platform& platform, Communicator& communicator, Logger& logger);

  VehicleSystem& getSystem();

private:
  UnitData data;
};

#endif
