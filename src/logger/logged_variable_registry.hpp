#ifndef LOGGED_VARIABLE_REGISTRY_HPP
#define LOGGED_VARIABLE_REGISTRY_HPP

#include <vector>

// forward declare
template <typename T>
class LoggedVariable;

class LoggedVariableRegistry {
public:
  void registerInt(LoggedVariable<int> *variable);
  void registerFloat(LoggedVariable<float> *variable);

private:
  std::vector<LoggedVariable<int> *> ints;
  std::vector<LoggedVariable<float> *> floats;
};

#endif // LOGGED_VARIABLE_REGISTRY_HPP
