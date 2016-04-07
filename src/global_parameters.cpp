#include "global_parameters.hpp"

GlobalParameters::GlobalParameters(ParameterRepository& params) {
  params.def(PARAM_DT, 1.0 / 1000.0);
}
