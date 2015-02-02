#include "variant/unit.hpp"

Unit::Unit(Platform& platform, Communicator& communicator)
  : data(platform, params, communicator) {

  params.insert<float>("DT", 0.001);
}

VehicleSystem& Unit::getSystem() {
  return data.system;
}
