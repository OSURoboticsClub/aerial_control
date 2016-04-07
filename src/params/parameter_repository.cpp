#include "params/parameter_repository.hpp"

void ParameterRepository::def(char const *name, const float value) {
    if(parameters.count(name) == 0) {
        parameters[name] = value;
    }
}

void ParameterRepository::set(char const *name, const float value) {
    parameters[name] = value;
}

float ParameterRepository::get(char const *name) const {
    return parameters.at(name);
}
