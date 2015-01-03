#ifndef UNIT_HPP_
#define UNIT_HPP_

#include "unit_data.hpp"
#include "system/vehicle_system.hpp"
#include "variant/platform.hpp"

class Unit {
public:
  Unit(Platform& platform);

  void init();

  VehicleSystem& getSystem();

private:
  unit_data_t data;
};

#endif
