#ifndef SDC_DEVICE_HPP_
#define SDC_DEVICE_HPP_

#include <hal.h>

class SDCDevice {
public:
  explicit SDCDevice(SDCDriver *sdcd, const SDCConfig *sdccfg);

protected:
  SDCDriver *sdcd;
  const SDCConfig *sdccfg;
};

#endif // SDC_DEVICE_HPP_
