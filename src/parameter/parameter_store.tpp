#include <cstring>

template <typename T>
void ParameterStore::insert(const char *name, T value) {
  Parameter<T> *param = static_cast<Parameter<T> *>(chCoreAlloc(sizeof(Parameter<T>)));
  *param = Parameter<T>(name, value);

  insert(param);
}

template <typename T>
Parameter<T> *ParameterStore::find(const char *name) const {
  // TODO(kyle): Linear search is probably slower than this should be,
  // especially if config values are being looked up each control loop.
  //
  // Consider using a map.
  for(BaseParameter *param : parameters) {
    if(param && std::strcmp(param->getName(), name) == 0) {
      return static_cast<Parameter<T> *>(param);
    }
  }

  return nullptr;
}
