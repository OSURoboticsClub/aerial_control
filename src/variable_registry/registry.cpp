#include "variable_registry/registry.hpp"

void VariableRegistry::registerInt(RegisteredVariable<int> *variable) {
  ints.push_back(variable);
}

void VariableRegistry::registerFloat(RegisteredVariable<float> *variable) {
  floats.push_back(variable);
}

