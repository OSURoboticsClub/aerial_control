#ifndef ICU_PLATFORM_HPP_
#define ICU_PLATFORM_HPP_

#include <cstdint>

class ICUPlatform {
public:
  ICUPlatform();

  /**
   * Get number of pulses in last capture
   */
  int getCaptureCount(void);

private:
  int captureCount;
  std::array<float, 20> widths;
  std::array<float, 20> periods;
};

#endif
