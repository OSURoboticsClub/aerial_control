#include "variant/indicator_platform.hpp"

#include <hal.h>

IndicatorPlatform::IndicatorPlatform() {
}

void IndicatorPlatform::set(IndicatorIntent intent, bool value) {
  switch(intent) {
    case HEARTBEAT:
      palWritePad(GPIOE, GPIOE_LED3_RED, value);
      break;
    case VEHICLE_ARMED:
      palWritePad(GPIOE, GPIOE_LED4_BLUE, value);
      break;
  }
}
