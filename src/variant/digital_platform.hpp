#ifndef DIGITAL_PLATFORM_HPP_
#define DIGITAL_PLATFORM_HPP_

#include <cstdint>

class DigitalPlatform {
public:
  /**
   * Get the singleton instance.
   */
  static DigitalPlatform& getInstance() {
    static DigitalPlatform platform;
    return platform;
  }

  void set(std::uint8_t ch, bool on);

private:
  DigitalPlatform();

};

#endif
