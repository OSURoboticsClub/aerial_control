#ifndef PARAMETER_STORE_HPP_
#define PARAMETER_STORE_HPP_

#include <array>
#include <cstring>
#include <cstddef>

#include "parameter/base_parameter.hpp"
#include "parameter/parameter.hpp"

class ParameterStore {
public:
  ParameterStore();

  void insert(BaseParameter *param);

  template <typename T>
  Parameter<T> *find(const char *name) const;

private:
  std::size_t pos;
  std::array<BaseParameter *, 100> parameters;
};

#include "parameter/parameter_store.tpp"

#endif
