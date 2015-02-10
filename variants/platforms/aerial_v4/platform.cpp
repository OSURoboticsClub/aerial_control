#include "variant/platform.hpp"

#include "hal.h"

#include "drivers/mpu9250.hpp"
#include "sensor/accelerometer.hpp"
#include "sensor/gyroscope.hpp"

#include "drivers/vishaytherm.hpp"
#include "sensor/thermistor.hpp"

#include "drivers/ublox_neo7.hpp"
#include "sensor/gps.hpp"

#include "variant/adc_platform.hpp"
#include "variant/i2c_platform.hpp"
#include "variant/pwm_platform.hpp"
#include "variant/spi_platform.hpp"
#include "variant/usart_platform.hpp"

Platform::Platform() {
}

// MS5611 SPI configuration
// TODO(yoos): verify clock speed
static const SPIConfig MS5611_CONFIG {
  NULL,
  GPIOC,
  13,
  SPI_CR1_BR_0   // 42000000/2^1 = 21000000
};

// MPU9250 SPI configuration
static const SPIConfig MPU9250_CONFIG {
  NULL,
  GPIOC,
  14,
  SPI_CR1_BR_0   // 42000000/2^1 = 21000000
};

// H3LIS331DL SPI configuration
// TODO(yoos): verify clock speed
static const SPIConfig H3LIS331DL_CONFIG {
  NULL,
  GPIOC,
  15,
  SPI_CR1_BR_0   // 42000000/2^1 = 21000000
};

const size_t ADC_GRP1_NUM_CHANNELS = 2;
const size_t ADC_GRP1_BUF_DEPTH = 4;

static std::array<adcsample_t, ADC_GRP1_NUM_CHANNELS*ADC_GRP1_BUF_DEPTH> samples;
static std::array<adcsample_t, ADC_GRP1_NUM_CHANNELS> avg_ch;

static void adccb(ADCDriver *adcp, adcsample_t *buffer, std::size_t n) {
  (void) buffer; (void) n;
  // NOTE: Update avg_ch only in the ADC_COMPLETE state because the ADC driver
  // fires an intermediate callback when the buffer is half full.
  if (adcp->state == ADC_COMPLETE) {
    for (std::size_t i=0; i<ADC_GRP1_NUM_CHANNELS; i++) {
      avg_ch[i] = (samples[i] + samples[i+1]) / ADC_GRP1_BUF_DEPTH;
    }
  }
}

// ADC configuration
// TODO(yoos): copy over comments from osuar_control repo.
static const ADCConversionGroup ADC_GRP1_CONFIG {
  FALSE,   // Linear buffer (TRUE for circular)
  ADC_GRP1_NUM_CHANNELS,   // Number of analog channels belonging to this group
  adccb,   // Callback (NULL if none)
  NULL,    // Error callback (NULL if none)

  // HW dependent part
  0,                 // CR1 initialization data
  ADC_CR2_SWSTART,   // CR2 Initialization data
  ADC_SMPR1_SMP_AN10(ADC_SAMPLE_56) | ADC_SMPR1_SMP_AN11(ADC_SAMPLE_56),
  0,
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),   // SQR1 init data
  0,                                        // SQR2 init data
  ADC_SQR3_SQ2_N(ADC_CHANNEL_IN11)          // SQR3 init data
};

template <>
MPU9250& Platform::get() {
  static MPU9250 imu(&SPID1, &MPU9250_CONFIG);
  return imu;
}

template <>
Gyroscope& Platform::get() {
  return get<MPU9250>();
}

template <>
Accelerometer& Platform::get() {
  return get<MPU9250>();
}

template <>
VishayTherm& Platform::get() {
  static VishayTherm therm(&ADCD1, &ADC_GRP1_CONFIG, &samples, &avg_ch);
  return therm;
}

template <>
ADCPlatform& Platform::get() {
  static ADCPlatform adcPlatform;
  return adcPlatform;
}

template <>
UBloxNEO7& Platform::get() {
  static UBloxNEO7 gps(&SD6);
  return gps;
}

template <>
GPS& Platform::get() {
  return get<UBloxNEO7>();
}

template <>
I2CPlatform& Platform::get() {
  static I2CPlatform i2cPlatform;
  return i2cPlatform;
}

template <>
PWMPlatform& Platform::get() {
  static PWMPlatform pwmPlatform;
  return pwmPlatform;
}

template <>
SPIPlatform& Platform::get() {
  static SPIPlatform spiPlatform;
  return spiPlatform;
}

template <>
USARTPlatform& Platform::get() {
  static USARTPlatform usartPlatform;
  return usartPlatform;
}

void Platform::init() {
  get<ADCPlatform>();
  get<I2CPlatform>();
  get<PWMPlatform>();
  get<SPIPlatform>();
  get<USARTPlatform>();

  // Initialize IMU
  get<MPU9250>().init();

  // Initialize thermistor
  get<VishayTherm>().init();

  // Initialize GPS
  get<UBloxNEO7>().init();
}
