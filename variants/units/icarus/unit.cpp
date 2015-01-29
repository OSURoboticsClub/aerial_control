#include "variant/unit.hpp"

Unit::Unit(Platform& platform, Communicator& communicator)
  : data(platform.get<Gyroscope>(), platform.get<Accelerometer>(),
    platform.get<PWMPlatform>(), communicator) {
}

VehicleSystem& Unit::getSystem() {
  return data.system;
}
