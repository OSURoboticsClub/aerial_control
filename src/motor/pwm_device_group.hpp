#ifndef PWM_DEVICE_GROUP_HPP_
#define PWM_DEVICE_GROUP_HPP_

#include <array>

#include "hal.h"

#include "controller/setpoint_types.hpp"
#include "variant/pwm_platform.hpp"

/** 
 * A grouping of PWM channels that are scaled relative to each other.
 */
template <int device_count>
class PWMDeviceGroup {
public:
  /**
   * Construct a new PWMDeviceGroup to output on the specified channels. Outputs
   * are mapped from the input range to the output range, then offset.
   *
   * If the system is disarmed then devices are set to the safe value.
   */
  PWMDeviceGroup(PWMPlatform& pwmPlatform, std::array<pwmchannel_t, device_count> channels,
      std::array<float, device_count> offsets, float inMin, float inMax,
      float outMin, float outMax, float outSafe);

  void set(bool armed, const std::array<float, device_count>& percents);

private:
  PWMPlatform& pwmPlatform;

  std::array<pwmchannel_t, device_count> channels;
  std::array<float, device_count> offsets;

  float inMin, inMax;
  float outMin, outMax;
  float outSafe;
};

#include "motor/pwm_device_group.tpp"

#endif
