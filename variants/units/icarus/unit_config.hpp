#ifndef UNIT_CONFIG_HPP_
#define UNIT_CONFIG_HPP_

#include <array>

namespace unit_config {

// Sensor offsets
const std::array<int, 3>   GYR_AXES  = {2, 3, 1};
const std::array<int, 3>   ACC_AXES  = {2, 3, 1};
const std::array<int, 3>   ACCH_AXES = {1, 2, 3};
const std::array<float, 3> GYR_OFFSETS  = {0,0,0};
const std::array<float, 3> ACC_OFFSETS  = {0,0,0};
const std::array<float, 3> ACCH_OFFSETS = {0,0,0};

}

#endif // UNIT_CONFIG_HPP_
