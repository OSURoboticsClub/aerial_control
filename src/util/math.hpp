#ifndef MATH_HPP
#define MATH_HPP

#include <algorithm>

// Use preprocessor define to be backwards-compatible with old M_PI definitions.
#define M_PI 3.14159265358979323846

template <typename T>
T clip(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}

#endif
