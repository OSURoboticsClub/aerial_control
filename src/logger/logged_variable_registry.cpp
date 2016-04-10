#include "logger/logged_variable_registry.hpp"

void LoggedVariableRegistry::registerInt(LoggedVariable<int> *variable) {
  ints.push_back(variable);
}

void LoggedVariableRegistry::registerFloat(LoggedVariable<float> *variable) {
  floats.push_back(variable);
}

