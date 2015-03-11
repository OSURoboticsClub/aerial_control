#ifndef PLATFORM_HPP_
#define PLATFORM_HPP_

#include <cstdint>

class Platform {
public:
  Platform();

  void init();

  template <typename T>
  T& get(int idx=0);
};

#endif
