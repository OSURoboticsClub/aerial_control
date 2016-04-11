#include "variant/unit.hpp"

Unit::Unit(Platform& platform, ParameterRepository& params, Communicator& communicator)
  : data(platform, params, communicator) {
}

VehicleSystem& Unit::getSystem() {
  return data.system;
}
