#ifndef VARIABLE_REGISTRY_REGISTRY_HPP
#define VARIABLE_REGISTRY_REGISTRY_HPP

#include <vector>

// forward declare
template <typename T>
class RegisteredVariable;

class VariableRegistry {
public:
  void registerInt(RegisteredVariable<int> *variable);
  void registerFloat(RegisteredVariable<float> *variable);

private:
  std::vector<RegisteredVariable<int> *> ints;
  std::vector<RegisteredVariable<float> *> floats;
};

#endif // VARIABLE_REGISTRY_REGISTRY_HPP
