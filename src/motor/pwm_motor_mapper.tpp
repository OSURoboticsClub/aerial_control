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
  return (a - inputRangeMin) * scale + outputRangeMin;
}

template <int motor_count>
void PWMMotorMapper<motor_count>::setMotorSpeeds(bool armed, int chanOff, const std::array<float, motor_count>& percents, float rangeMin, float rangeMax) {
  if(armed) {
    float smin = std::min(rangeMin, *std::min_element(percents.begin(), percents.end()));
    float smax = std::max(rangeMax, *std::max_element(percents.begin(), percents.end()));

    for(std::size_t i = 0; i < motor_count; i++) {
      float zo = map(smin, smax, rangeMin, rangeMax, percents[i]);
      float pwm = map(rangeMin, rangeMax, unit_config::THROTTLE_MIN, unit_config::THROTTLE_MAX, zo);

      pwmPlatform.set(i + chanOff, pwm);
    }
  } else {
    for(std::size_t i = 0; i < motor_count; i++) {
      pwmPlatform.set(i + chanOff, unit_config::THROTTLE_SAFE);
    }
  }
}
