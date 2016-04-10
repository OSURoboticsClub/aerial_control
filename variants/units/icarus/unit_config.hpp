#ifndef UNIT_CONFIG_HPP_
#define UNIT_CONFIG_HPP_

#include <array>

namespace unit_config {

const float DT = 0.001;

const double PI = 3.14159265358979323846;

// Maximum angular position in the roll and pitch axes (rad)
const float MAX_PITCH_ROLL_POS = 30.0 * PI / 180.0;

// Maximum angular velocity in the roll and pitch axes (rad/s)
const float MAX_PITCH_ROLL_VEL = 200.0 * PI / 180.0;

// Maximum angular acceleration (rad/s^2)
const float MAX_PITCH_ROLL_ACC = 100.0 * PI / 180.0;   // TODO: calculate properly

// Sensor offsets
const std::array<int, 3>   GYR_AXES  = {3, 2, -1};
const std::array<int, 3>   ACC_AXES  = {3, 2, -1};
const std::array<int, 3>   ACCH_AXES = {-3, -2, -1};
const std::array<float, 3> GYR_OFFSETS  = {0.023, 0.022, 0.007};
const std::array<float, 3> ACC_OFFSETS  = {0.105, -0.005, 0.020};
const std::array<float, 3> ACCH_OFFSETS = {0,0,0};

// Initial angular position controller gains
const float ANGPOS_X_KP = 1.0;
const float ANGPOS_X_KI = 0.0;
const float ANGPOS_X_KD = 0.0;
const float ANGPOS_Y_KP = 1.0;
const float ANGPOS_Y_KI = 0.0;
const float ANGPOS_Y_KD = 0.0;
const float ANGPOS_Z_KP = 1.0;
const float ANGPOS_Z_KI = 0.0;
const float ANGPOS_Z_KD = 0.0;

// Initial angular velocity controller gains
const float ANGVEL_X_KP = 1.0;   // TODO(yoos): Eyeballed on ground. Update after flight.
const float ANGVEL_X_KI = 0.0;
const float ANGVEL_X_KD = 0.0;
const float ANGVEL_Y_KP = 1.0;
const float ANGVEL_Y_KI = 0.0;
const float ANGVEL_Y_KD = 0.0;
const float ANGVEL_Z_KP = 1.0;
const float ANGVEL_Z_KI = 0.0;
const float ANGVEL_Z_KD = 0.0;

// Initial angular acceleration controller gains
const float ANGACC_X_KP = 1.0;
const float ANGACC_X_KI = 0.0;
const float ANGACC_X_KD = 0.0;
const float ANGACC_Y_KP = 1.0;
const float ANGACC_Y_KI = 0.0;
const float ANGACC_Y_KD = 0.0;
const float ANGACC_Z_KP = 1.0;
const float ANGACC_Z_KI = 0.0;
const float ANGACC_Z_KD = 0.0;

}

#endif // UNIT_CONFIG_HPP_
