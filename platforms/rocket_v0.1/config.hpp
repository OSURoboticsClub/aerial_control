#ifndef CONFIG_HPP_
#define CONFIG_HPP_

#include <hal.h>

#include <unit_config.hpp>

const float DT = 0.001;

// Motor PWM configuration
const PWMConfig motor_pwm_config = {
  500000,    // 500 kHz PWM clock frequency.
  1000,      // PWM period 2.0 ms.
  NULL,      // No callback.
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },   // Channel configurations
  0,0   // HW dependent
};


// L3GD20 SPI configuration
// TODO(yoos): This is actually MPU-6000 config from last year.
const SPIConfig l3gd20_spi_config = {
  NULL,
  GPIOB,
  2,
  SPI_CR1_BR_0   // 42000000/2^1 = 21000000
};

// I2C1 configuration
const I2CConfig lsm303dlhc_i2c_config = {
  OPMODE_I2C,
  400000,
  FAST_DUTY_CYCLE_2
};

// USART1 configuration
const SerialConfig usart1_config = {
  115200,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};

#endif
