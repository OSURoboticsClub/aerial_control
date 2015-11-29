#include "variant/indicator_platform.hpp"

#include <hal.h>

IndicatorPlatform::IndicatorPlatform() {
}

void IndicatorPlatform::set(IndicatorIntent intent, bool value) {
  switch(intent) {
    case HEARTBEAT:
      palWritePad(GPIOB, 2, value);
      break;
    case VEHICLE_ARMED:
      palWritePad(GPIOA, 15, value);
      break;
  }
}
