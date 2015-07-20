#ifndef INDICATOR_PLATFORM_HPP_
#define INDICATOR_PLATFORM_HPP_

enum IndicatorIntent {
  HEARTBEAT,
  VEHICLE_ARMED
};

class IndicatorPlatform {
public:
  /**
   * Get the singleton instance.
   */
  static IndicatorPlatform& getInstance() {
    static IndicatorPlatform platform;
    return platform;
  }

  void set(IndicatorIntent intent, bool value);

private:
  IndicatorPlatform();

  IndicatorPlatform(IndicatorPlatform& platform) = delete;
  void operator=(IndicatorPlatform& platform) = delete;
};

#endif
