#include "variant/i2c_platform.hpp"

#include "hal.h"

// See ChibiOS/os/hal/platforms/STM32/I2Cv1
static const I2CConfig I2CD2_CONFIG = {
	OPMODE_I2C,
	400000,
	FAST_DUTY_CYCLE_2
};

I2CPlatform::I2CPlatform() {
	// I2C2
	i2cStart(&I2CD2, &I2CD2_CONFIG);
	palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(4));   // SCL
	palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(4));   // SDA
}
