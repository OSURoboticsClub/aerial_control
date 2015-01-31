#ifndef UNIT_CONFIG_HPP_
#define UNIT_CONFIG_HPP_

namespace unit_config {

const float DT = 0.001;

// Maximum angular position in the roll and pitch axes (rad)
const float MAX_PITCH_ROLL_POS = 10.0;   // No limit

// Maximum angular velocity in the roll and pitch axes (rad/s)
const float MAX_PITCH_ROLL_VEL = 2.0;

// Maximum angular acceleration (rad/s^2)
const float MAX_PITCH_ROLL_ACC = 4.0;   // TODO: calculate properly

// Sensor offsets
const float GYR_X_OFFSET = 0.016;
const float GYR_Y_OFFSET = 0.021;
const float GYR_Z_OFFSET = -0.002;
const float ACC_X_OFFSET = -0.005;
const float ACC_Y_OFFSET = -0.020;
const float ACC_Z_OFFSET = -0.071;

// Initial angular position controller gains
const float ANGPOS_X_KP = 0.0;
const float ANGPOS_X_KI = 0.0;
const float ANGPOS_X_KD = 0.0;
const float ANGPOS_Y_KP = 0.2;
const float ANGPOS_Y_KI = 0.0;
const float ANGPOS_Y_KD = 0.0;
const float ANGPOS_Z_KP = 0.2;
const float ANGPOS_Z_KI = 0.0;
const float ANGPOS_Z_KD = 0.0;

// Initial angular velocity controller gains
const float ANGVEL_X_KP = -1.0;
const float ANGVEL_X_KI = 0.0;
const float ANGVEL_X_KD = 0.0;
const float ANGVEL_Y_KP = 0.0;
const float ANGVEL_Y_KI = 0.0;
const float ANGVEL_Y_KD = 0.0;
const float ANGVEL_Z_KP = 0.0;
const float ANGVEL_Z_KI = 0.0;
const float ANGVEL_Z_KD = 0.0;

// Initial angular acceleration controller gains
const float ANGACC_X_KP = 0.2;
const float ANGACC_X_KI = 0.0;
const float ANGACC_X_KD = 0.0;
const float ANGACC_Y_KP = 0.0;
const float ANGACC_Y_KI = 0.0;
const float ANGACC_Y_KD = 0.0;
const float ANGACC_Z_KP = 0.0;
const float ANGACC_Z_KI = 0.0;
const float ANGACC_Z_KD = 0.0;

}

#endif // UNIT_CONFIG_HPP_
