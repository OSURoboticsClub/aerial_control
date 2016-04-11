#pragma once

#include <array>
#include <utility>

class ParameterRepository {
public:
  constexpr static std::size_t MAX_PARAMETERS = 100;

  /**
   * Inserts a default value into the repository, if an actual value has not
   * yet been inserted. This allows modules to establish default values that
   * will only be used if a specific parameter value was not set in a previous
   * configuration step.
   *
   * @param name the parameter name
   * @param value the default value
   */
  void def(char const *name, const float value);

  /**
   * Insert a value into the repository, overwriting any existing value.
   *
   * @param name the parameter name
   * @param value the new value
   */
  void set(char const *name, const float value);

  /**
   * Grabs a value from the repository. If the key does not exist then an
   * exception is thrown.
   *
   * @param name the parameter name
   * @return the set value
   */
  float get(char const *name) const;

private:
  std::array<std::pair<char const*, float>, MAX_PARAMETERS> parameters;
  std::size_t parameterIdx = 0;
};
