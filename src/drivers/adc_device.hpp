#ifndef ADC_DEVICE_HPP_
#define ADC_DEVICE_HPP_

#include <array>
#include <cstdint>

#include "hal.h"

template <std::size_t num_ch, std::size_t buf_depth>
class ADCDevice {
public:
  ADCDevice(ADCDriver *adcd, const ADCConversionGroup *adcgrpcfg, std::array<adcsample_t, num_ch*buf_depth> *samples, std::array<adcsample_t, num_ch> *avg_ch);

protected:
  void update(void);

  ADCDriver *adcd;
  const ADCConversionGroup *adcgrpcfg;
  std::array<adcsample_t, num_ch*buf_depth> *samples;
  std::array<adcsample_t, num_ch> *avg_ch;
};

#include "drivers/adc_device.tpp"

#endif // ADC_DEVICE_HPP_
