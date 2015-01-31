#include "parameter/base_parameter.hpp"

#include <cstring>

#include "ch.hpp"

BaseParameter::BaseParameter(const char *name_) {
  // Copy name
  name = static_cast<char *>(chCoreAlloc(std::strlen(name_)));
  std::strcpy(name, name_);
}

char *BaseParameter::getName() const {
  return name;
}
