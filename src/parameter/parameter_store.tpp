#include <cstring>

template <typename T>
Parameter<T> *ParameterStore::find(const char *name) const {
  for(auto *param : parameters) {
    if(param && std::strcmp(param->getName(), name) == 0) {
      return static_cast<Parameter<T> *>(param);
    }
  }

  return nullptr;
}
