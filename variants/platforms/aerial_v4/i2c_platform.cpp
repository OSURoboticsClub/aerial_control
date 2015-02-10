#include "variant/i2c_platform.hpp"

#include "hal.h"

I2CPlatform::I2CPlatform() {
	// I2C2
	palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(4));   // SCL
	palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(4));   // SDA
}
