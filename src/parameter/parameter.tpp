#include "ch.hpp"

template <typename T>
Parameter<T>::Parameter(const char *name, T value_) : BaseParameter(name) {
  // Copy value
  value = static_cast<T *>(chCoreAlloc(sizeof(T)));
  *value = value_;
}

template <typename T>
T Parameter<T>::get() {
  return *value;
}

template <typename T>
void Parameter<T>::put(T value) {
  *value = value;
}
