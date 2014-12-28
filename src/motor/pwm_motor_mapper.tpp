#include <algorithm>
#include <limits>

#include "hal_config.hpp"

template <int motor_count>
PWMMotorMapper<motor_count>::PWMMotorMapper() {
}

template <int motor_count>
void PWMMotorMapper<motor_count>::init() {
}

template <int motor_count>
void PWMMotorMapper<motor_count>::setMotorSpeeds(const std::array<float, motor_count>& percents) {
  std::array<float, motor_count> mappedPercents;
  mapToBounds(percents, &mappedPercents);

  for(int i = 0; i < motor_count; i++) {
    pwmPlatformSet(i, mappedPercents[i]);
  }
}

template <int motor_count>
void PWMMotorMapper<motor_count>::mapToBounds(const std::array<float, motor_count>& percents, std::array<float, motor_count> *mapped) {
  // TODO: Make these configurable
  float output_min = 0.0f, output_max = 0.7f;

  // Find the minimum and maximum inputs
  float input_min = *std::min_element(percents.begin(), percents.end());
  float input_max = *std::max_element(percents.begin(), percents.end());

  // Limit the outputs to the maximum values
  float scale = (output_max - output_min) / (input_max - input_min);
  if(scale < 1.0f) {
    for(int i = 0; i < motor_count; i++) {
      (*mapped)[i] = (percents[i] - input_min) * scale + output_min;
    }
  }
}
