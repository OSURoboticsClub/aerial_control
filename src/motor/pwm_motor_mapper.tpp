#include <algorithm>
#include <cstddef>
#include <limits>

#include "protocol/messages.hpp"

#include "unit_config.hpp"

template <int motor_count>
PWMMotorMapper<motor_count>::PWMMotorMapper(PWMPlatform& pwmPlatform)
  : pwmPlatform(pwmPlatform) {
}

template <int motor_count>
void PWMMotorMapper<motor_count>::init() {
}

static float map(float inputRangeMin, float inputRangeMax, float outputRangeMin, float outputRangeMax, float a) {
  float scale = (outputRangeMax - outputRangeMin) / (inputRangeMax - inputRangeMin);
  return (a - inputRangeMax) * scale + outputRangeMax;
}

template <int motor_count>
void PWMMotorMapper<motor_count>::setMotorSpeeds(bool armed, const std::array<float, motor_count>& percents) {
  if(armed) {
    float smin = std::min(0.0f, *std::min_element(percents.begin(), percents.end()));
    float smax = std::max(1.0f, *std::max_element(percents.begin(), percents.end()));

    for(std::size_t i = 0; i < motor_count; i++) {
      float zo = map(smin, smax, 0.0f, 1.0f, percents[i]);
      float pwm = map(0.0f, 1.0f, unit_config::THROTTLE_MIN, unit_config::THROTTLE_MAX, zo);

      pwmPlatform.set(i, pwm);
    }
  } else {
    for(std::size_t i = 0; i < motor_count; i++) {
      pwmPlatform.set(i, unit_config::THROTTLE_SAFE);
    }
  }
}
