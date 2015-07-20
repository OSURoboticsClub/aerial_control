#ifndef PWM_PLATFORM_HPP_
#define PWM_PLATFORM_HPP_

#include <cstdint>

class PWMPlatform {
public:
  PWMPlatform();

  /**
   * Get the singleton instance.
   */
  static PWMPlatform& getInstance() {
    static PWMPlatform platform;
    return platform;
  }

  void set(std::uint8_t ch, float dc);
};

#endif
