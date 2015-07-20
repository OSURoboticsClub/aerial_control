#ifndef MATH_HPP
#define MATH_HPP

template <typename T>
T clip(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}

#endif
