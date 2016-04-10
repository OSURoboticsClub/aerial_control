#ifndef LOGGED_VARIABLE_HPP
#define LOGGED_VARIABLE_HPP

#include "logger/logged_variable_registry.hpp"

template <typename T>
class LoggedVariable {
public:
  LoggedVariable(LoggedVariableRegistry& registry, T value);

  T v();   // Get value
  void set(T value);

private:
  T value;
};

template <>
LoggedVariable<int>::LoggedVariable(LoggedVariableRegistry& registry, int value) : value(value) {
  registry.registerInt(this);
}

template <>
LoggedVariable<double>::LoggedVariable(LoggedVariableRegistry& registry, double value) : value(value) {
  registry.registerDouble(this);
}

template <typename T>
T LoggedVariable<T>::v() {
  return value;
}

template <typename T>
void LoggedVariable<T>::set(T value) {
  this->value = value;
}

#endif // LOGGED_VARIABLE_HPP
