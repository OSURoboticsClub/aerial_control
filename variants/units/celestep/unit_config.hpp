#ifndef UNIT_CONFIG_HPP_
#define UNIT_CONFIG_HPP_

#include <array>

namespace unit_config {

// Sensor config
const std::array<int, 3>   GYR_AXES  = {-1,  2, -3};
const std::array<int, 3>   ACC_AXES  = {-1,  2, -3};
const std::array<int, 3>   ACCH_AXES = { 1, -2, -3};
const std::array<float, 3> GYR_OFFSETS  = {0.00834, -0.00471, 0.00461};
const std::array<float, 3> ACC_OFFSETS  = {-0.015, -0.018, 0.203};
const std::array<float, 3> ACCH_OFFSETS = {0.396, -0.635, 0.946};

}

#endif // UNIT_CONFIG_HPP_
