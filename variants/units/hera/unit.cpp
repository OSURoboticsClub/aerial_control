#include "variant/unit.hpp"

#include "sensor/accelerometer.hpp"
#include "sensor/gyroscope.hpp"

Unit::Unit(Platform& platform)
  : data(platform.get<Gyroscope>(), platform.get<Accelerometer>(), platform.get<PWMPlatform>()) {
}

void Unit::init() {
  data.system.init();
}

VehicleSystem& Unit::getSystem() {
  return data.system;
}
