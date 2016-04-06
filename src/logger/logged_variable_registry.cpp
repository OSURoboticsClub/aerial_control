#include "logger/logged_variable_registry.hpp"

void LoggedVariableRegistry::registerInt(LoggedVariable<int> *variable) {
  ints.push_back(variable);
}

void LoggedVariableRegistry::registerDouble(LoggedVariable<double> *variable) {
  doubles.push_back(variable);
}

