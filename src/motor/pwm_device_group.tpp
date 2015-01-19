#include <algorithm>

#include "unit_config.hpp"

template <int device_count>
PWMDeviceGroup<device_count>::PWMDeviceGroup(PWMPlatform& pwmPlatform,
    std::array<pwmchannel_t, device_count> channels,
    std::array<float, device_count> offsets,
    float inMin, float inMax, float outMin, float outMax, float outSafe)
  : pwmPlatform(pwmPlatform), channels(channels), offsets(offsets), inMin(inMin),
    inMax(inMax), outMin(outMin), outMax(outMax), outSafe(outSafe) {
}

// TODO: map2 -> map
static float map2(float inputRangeMin, float inputRangeMax, float outputRangeMin, float outputRangeMax, float a) {
  float scale = (outputRangeMax - outputRangeMin) / (inputRangeMax - inputRangeMin);
  return (a - inputRangeMin) * scale + outputRangeMin;
}

template <int device_count>
void PWMDeviceGroup<device_count>::set(bool armed, const std::array<float, device_count>& percents) {
  if(armed) {
    float smin = std::min(inMin, *std::min_element(percents.begin(), percents.end()));
    float smax = std::max(inMax, *std::max_element(percents.begin(), percents.end()));

    for(std::size_t i = 0; i < device_count; i++) {
      float zo = map2(smin, smax, inMin, inMax, percents[i]);
      float pwm = map2(inMin, inMax, outMin, outMax, zo);

      pwmPlatform.set(channels[i], pwm + offsets[i]);
    }
  } else {
    for(std::size_t i = 0; i < device_count; i++) {
      pwmPlatform.set(channels[i], outSafe + offsets[i]); // TODO: Remove offset?
    }
  }
}
