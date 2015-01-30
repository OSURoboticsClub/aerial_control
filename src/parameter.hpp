#include <array>
#include <cstddef>
#include <cstring>

#include "ch.hpp"

class BaseParameter {
public:
  BaseParameter(const char *name_) {
    // Copy name
    name = static_cast<char *>(chCoreAlloc(std::strlen(name_)));
    std::strcpy(name, name_);
  }

  char *getName() const {
    return name;
  }

private:
  char *name;
};

template <typename T>
class Parameter : public BaseParameter {
public:
  Parameter(const char *name, T value_) : BaseParameter(name) {
    // Copy value
    value = static_cast<T *>(chCoreAlloc(sizeof(T)));
    *value = value_;
  }

  T get() {
    return *value;
  }

  void put(T value) {
    *value = value;
  }

private:
  T *value;
};

class ParameterStore {
public:
  void insert(BaseParameter *param) {
    parameters[pos++] = param;
  }

  BaseParameter *find(const char *name) const {
    for(auto *param : parameters) {
      if(param && std::strcmp(param->getName(), name) == 0) {
        return param;
      }
    }

    return nullptr;
  }
private:
  std::size_t pos;
  std::array<BaseParameter *, 100> parameters;
};
