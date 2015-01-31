#include "parameter/parameter_store.hpp"

#include <cstring>

ParameterStore::ParameterStore() : pos(0), parameters{nullptr} {
}

void ParameterStore::insert(BaseParameter *param) {
  parameters[pos++] = param;
}
