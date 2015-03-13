#ifndef PLATFORM_HPP_
#define PLATFORM_HPP_

#include <cstdint>

class Platform {
public:
  Platform();

  void init();

  template <typename T>
  T& get();

  template <typename T>
  T& getIdx(int idx);
};

#endif
