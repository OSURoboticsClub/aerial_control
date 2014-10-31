#include <algorithm>
#include <limits>
#include <tuple>

#include <config.hpp>

template <int motor_count>
PWMMotorMapper<motor_count>::PWMMotorMapper() {
  for(int i = 0; i < motor_count; i++) {
    channels[i] = i;
  }
}

template <int motor_count>
void PWMMotorMapper<motor_count>::init() {
  pwm = pwmPlatformInit();
}

template <int motor_count>
void PWMMotorMapper<motor_count>::setMotorSpeeds(const std::array<float, motor_count>& percents) {
  std::array<float, motor_count> mappedPercents = percents; // copy
  mapToBounds(mappedPercents, &mappedPercents);

  for(int i = 0; i < motor_count; i++) {
    pwmcnt_t dc = PWM_PERCENTAGE_TO_WIDTH(pwm, percents[i] * 10000);
    pwmEnableChannel(pwm, channels[i], dc);
  }
}

template <int motor_count>
void PWMMotorMapper<motor_count>::mapToBounds(std::array<float, motor_count>& percents, std::array<float, motor_count>* mapped) {
  // TODO: Make these configurable
  float output_min = 0.0f, output_max = 0.7f;

  // Find the minimum and maximum inputs
  float input_min = *std::min_element(percents.begin(), percents.end());
  float input_max = *std::max_element(percents.begin(), percents.end());

  // Limit the outputs to the maximum values
  float scale = (output_max - output_min) / (input_max - input_min);
  if(scale < 1.0f) {
    for(float& percent : *mapped) {
      percent = (percent - input_min) * scale + output_min;
    }
  }
}
