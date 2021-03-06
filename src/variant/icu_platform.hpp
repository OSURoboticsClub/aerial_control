#ifndef ICU_PLATFORM_HPP_
#define ICU_PLATFORM_HPP_

#include <functional>

#include <hal.h>

class ICUTriggerable {
public:
  virtual void trigger(ICUDriver *icup);
};

class ICUPlatform {
public:
  /**
   * Get the singleton instance.
   */
  static ICUPlatform& getInstance() {
    static ICUPlatform platform;
    return platform;
  }

  void registerTrigger(ICUTriggerable *instance) {
    instances[triggerIndex++] = instance;
  }

  void trigger(ICUDriver *icup) {
    for(unsigned int i = 0; i < triggerIndex; i++) {
      instances[i]->trigger(icup);
    }
  }

private:
  ICUPlatform();

  ICUPlatform(ICUPlatform& platform) = delete;
  void operator=(ICUPlatform& platform) = delete;

  static const unsigned int MAX_TRIGGERS = 1;

  unsigned int triggerIndex = 0;
  std::array<ICUTriggerable *, MAX_TRIGGERS> instances;
};

#endif
