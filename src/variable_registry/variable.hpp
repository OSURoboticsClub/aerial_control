#ifndef VARIABLE_REGISTRY_VARIABLE_HPP
#define VARIABLE_REGISTRY_VARIABLE_HPP

#include "variable_registry/registry.hpp"

template <typename T>
class RegisteredVariable {
public:
  RegisteredVariable(VariableRegistry& registry, T value);

  T v();   // Get value
  void set(T value);

private:
  T value;
};

template <>
RegisteredVariable<int>::RegisteredVariable(VariableRegistry& registry, int value) : value(value) {
  registry.registerInt(this);
}

template <>
RegisteredVariable<float>::RegisteredVariable(VariableRegistry& registry, float value) : value(value) {
  registry.registerFloat(this);
}

template <typename T>
T RegisteredVariable<T>::v() {
  return value;
}

template <typename T>
void RegisteredVariable<T>::set(T value) {
  this->value = value;
}

#endif // VARIABLE_REGISTRY_VARIABLE_HPP
