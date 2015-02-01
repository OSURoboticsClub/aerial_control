#ifndef UNIT_HPP_
#define UNIT_HPP_

#include "unit_data.hpp"

#include "communication/communicator.hpp"
#include "parameter/parameter_store.hpp"
#include "system/vehicle_system.hpp"
#include "variant/platform.hpp"

class Unit {
public:
  Unit(Platform& platform, Communicator& communicator);

  VehicleSystem& getSystem();

private:
  unit_data_t data;

  ParameterStore params;
};

#endif
