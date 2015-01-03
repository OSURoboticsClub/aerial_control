#ifndef PWM_PLATFORM_HPP_
#define PWM_PLATFORM_HPP_

#include <cstdint>

class PWMPlatform {
public:
  PWMPlatform();

  void set(std::uint8_t ch, float dc);
};

#endif
