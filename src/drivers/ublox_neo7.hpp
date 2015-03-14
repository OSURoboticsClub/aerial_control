#ifndef UBLOX_NEO7_HPP_
#define UBLOX_NEO7_HPP_

#include <cstdint>

#include "hal.h"

#include "drivers/usart_device.hpp"
#include "sensor/gps.hpp"

class UBloxNEO7 : protected USARTDevice<100, 100>, public GPS {
public:
  using USARTDevice::USARTDevice;

  void init() override;
  GPSReading readGPS() override;
  bool healthy();
};

#endif
