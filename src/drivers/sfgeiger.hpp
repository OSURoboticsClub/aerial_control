#ifndef SFGEIGER_HPP_
#define SFGEIGER_HPP_

#include <cstdint>

#include "hal.h"

#include "drivers/usart_device.hpp"
#include "sensor/geiger.hpp"

class SFGeiger : protected USARTDevice<100, 100>, public Geiger {
public:
  using USARTDevice::USARTDevice;

  void init() override;
  GeigerReading readGeiger() override;
  bool healthy() override;
};

#endif
