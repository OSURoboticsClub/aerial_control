#include <algorithm>
#include <limits>
#include <tuple>

#include <config.hpp>

template <int motor_count>
PWMMotorMapper<motor_count>::PWMMotorMapper() {
  // TODO: Make this configurable
  pwm = &PWMD8;
  for(int i = 0; i < motor_count; i++) {
    channels[i] = i;
  }
}

template <int motor_count>
void PWMMotorMapper<motor_count>::init() {
  pwmStart(pwm, &motor_pwm_config);
  palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(4));
  palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(4));
  palSetPadMode(GPIOC, 8, PAL_MODE_ALTERNATE(4));
  palSetPadMode(GPIOC, 9, PAL_MODE_ALTERNATE(4));
}

template <int motor_count>
void PWMMotorMapper<motor_count>::setMotorSpeeds(std::array<float, motor_count> percents) {
  for(int i = 0; i < motor_count; i++) {
    pwmcnt_t dc = PWM_PERCENTAGE_TO_WIDTH(pwm, percents[i] * 10000);
    pwmEnableChannel(pwm, channels[i], dc);
  }
}

template <int motor_count>
void PWMMotorMapper<motor_count>::mapToBounds(std::array<float, motor_count> percents) {
  // TODO: This will modify the `percents` parameter, perhaps unexpectedly

  float output_min = 0.0f, output_max = 0.7f;

  // Find the minimum and maximum inputs
  float input_min, input_max;
  // TODO: verify std::tie usage here
  std::tie(input_min, input_max) = std::minmax_element(percents.begin(), percents.end());

  // Limit the outputs to the maximum values
  float scale = (output_max - output_min) / (input_max - input_min);
  if(scale < 1.0f) {
    for(float& percent : percents) {
      percent = (percent - input_min) * scale + output_min;
    }
  }
}
