#ifndef VARIABLE_REGISTRY_REGISTRY_HPP
#define VARIABLE_REGISTRY_REGISTRY_HPP

#include <vector>

// forward declare
template <typename T>
class RegisteredVariable;
class VariableRegistrySubscriber;

class VariableRegistry {
public:
  void registerInt(RegisteredVariable<int> *variable);
  void registerFloat(RegisteredVariable<float> *variable);

  void registerSubscriber(VariableRegistrySubscriber *sub);

private:
  std::vector<RegisteredVariable<int> *> ints;
  std::vector<RegisteredVariable<float> *> floats;

  std::vector<VariableRegistrySubscriber*> subscribers;
};

#endif // VARIABLE_REGISTRY_REGISTRY_HPP
