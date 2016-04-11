#include "params/parameter_repository.hpp"

void ParameterRepository::def(char const *name, const float value) {
  // Only write if there isn't an existing value.
  if(parameters.count(name) == 0) {
    parameters[name] = value;
  }
}

void ParameterRepository::set(char const *name, const float value) {
  // Overwrite any existing value.
  parameters[name] = value;
}

float ParameterRepository::get(char const *name) const {
  return parameters.at(name);
}
