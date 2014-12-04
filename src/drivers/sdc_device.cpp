#include <drivers/sdc_device.hpp>

SDCDevice::SDCDevice(SDCDriver *sdcd, const SDCConfig *sdccfg) : sdcd(sdcd), sdccfg(sdccfg) {
}
