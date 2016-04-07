#ifndef GLOBAL_PARAMETERS_HPP_
#define GLOBAL_PARAMETERS_HPP_

#include "params/parameter_repository.hpp"

class GlobalParameters {
public:
  static constexpr char const *PARAM_DT = "global.dt";

  GlobalParameters(ParameterRepository& params);
};

#endif
