#ifndef DIGITAL_PLATFORM_HPP_
#define DIGITAL_PLATFORM_HPP_

#include <cstdint>

class DigitalPlatform {
public:
  DigitalPlatform();

  void set(std::uint8_t ch, bool on);
};

#endif
