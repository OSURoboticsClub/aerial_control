template <std::size_t num_ch, std::size_t buf_depth>
ADCDevice<num_ch, buf_depth>::ADCDevice(ADCDriver *adcd, const ADCConversionGroup *adcgrpcfg, std::array<adcsample_t, num_ch*buf_depth> *samples, std::array<adcsample_t, num_ch> *avg_ch)
  : adcd(adcd), adcgrpcfg(adcgrpcfg), samples(samples), avg_ch(avg_ch) {
}

template <std::size_t num_ch, std::size_t buf_depth>
void ADCDevice<num_ch, buf_depth>::update(void) {
  chSysLockFromIsr();
  adcStartConversionI(adcd, adcgrpcfg, reinterpret_cast<adcsample_t*>(samples), buf_depth);
  chSysUnlockFromIsr();
}
