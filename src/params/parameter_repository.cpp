#include "params/parameter_repository.hpp"

void ParameterRepository::def(char const *name, const float value) {
  // If a value already exists for this key, don't overwrite it with a default.
  for(std::size_t i = 0; i < parameterIdx; i++) {
    if(parameters[i].first == name) {
      return;
    }
  }

  // Otherwise, push it onto the end of the parameter list.
  parameters[parameterIdx++] = std::make_pair(name, value);
}

void ParameterRepository::set(char const *name, const float value) {
  for(std::size_t i = 0; i < parameterIdx; i++) {
    if(parameters[i].first == name) {
      parameters[i] = std::make_pair(name, value);
      break;
    }
  }

  parameters[parameterIdx++] = std::make_pair(name, value);
}

float ParameterRepository::get(char const *name) const {
  for(std::size_t i = 0; i < parameterIdx; i++) {
    if(parameters[i].first == name) {
      return parameters[i].second;
    }
  }

  // TODO: Error?
  return 0.0;
}
