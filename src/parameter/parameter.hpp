#ifndef PARAMETER_HPP_
#define PARAMETER_HPP_

#include "parameter/base_parameter.hpp"

template <typename T>
class Parameter : public BaseParameter {
public:
  Parameter(const char *name, T value_);

  /**
   * Return a copy of the parameter's contained value.
   */
  T get();

  /**
   * Update the value of the parameter.
   */
  void put(T value);

private:
  T *value;
};

#include "parameter/parameter.tpp"

#endif
