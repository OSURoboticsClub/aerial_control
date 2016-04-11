#ifndef UNIT_CONFIG_HPP_
#define UNIT_CONFIG_HPP_

#include <array>

namespace unit_config {

// Sensor config
const std::array<int, 3>   GYR_AXES  = {1, 2, 3};
const std::array<int, 3>   ACC_AXES  = {1, 2, 3};
const std::array<int, 3>   ACCH_AXES = {1, 2, 3};
const std::array<float, 3> GYR_OFFSETS  = {0.002752, -0.018451, -0.016139};
const std::array<float, 3> ACC_OFFSETS  = {0.0046, 0.0143, -0.0317};
const std::array<float, 3> ACCH_OFFSETS = {0.0, 0.0, 0.0};

}

#endif // UNIT_CONFIG_HPP_
