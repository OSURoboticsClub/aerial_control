#ifndef PARAMETER_STORE_HPP_
#define PARAMETER_STORE_HPP_

#include <array>
#include <cstddef>

#include "parameter/base_parameter.hpp"
#include "parameter/parameter.hpp"

class ParameterStore {
public:
  ParameterStore();

  /**
   * Insert a pre-allocated parameter.
   *
   * NOTE: The argument MUST outlive the ParameterStore.
   */
  void insert(BaseParameter *param);

  /**
   * Automatically allocate a new parameter and insert it into the store.
   */
  template <typename T>
  void insert(const char *name, T value);

  template <typename T>
  Parameter<T> *find(const char *name) const;

private:
  std::size_t pos;

  // TODO(kyle): Fixed size array is probably a bad idea.
  std::array<BaseParameter *, 100> parameters;
};

#include "parameter/parameter_store.tpp"

#endif
