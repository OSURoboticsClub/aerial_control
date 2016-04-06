#include "params/parameter_repository.hpp"

void ParameterRepository::def(char const *name, const double value) {
    if(parameters.count(name) == 0) {
        parameters[name] = value;
    }
}

void ParameterRepository::set(char const *name, const double value) {
    parameters[name] = value;
}

double ParameterRepository::get(char const *name) const {
    return parameters.at(name);
}
