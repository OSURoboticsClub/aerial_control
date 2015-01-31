#include "controller/rocket_angular_acceleration_controller.hpp"

#include <algorithm>

#include "unit_config.hpp"

RocketAngularAccelerationController::RocketAngularAccelerationController()
  : rollAccPid(unit_config::ANGACC_X_KP, unit_config::ANGACC_X_KI, unit_config::ANGACC_X_KD),
    pitchAccPid(unit_config::ANGACC_Y_KP, unit_config::ANGACC_Y_KI, unit_config::ANGACC_Y_KD),
    yawAccPid(unit_config::ANGACC_Z_KP, unit_config::ANGACC_Z_KI, unit_config::ANGACC_Z_KD) {
}

actuator_setpoint_t RocketAngularAccelerationController::run(const attitude_estimate_t& estimate, const angular_acceleration_setpoint_t& input) {
  // Limit to maximum angular accelerations
  float rollAccSp = std::max(-unit_config::MAX_PITCH_ROLL_ACC, std::min(unit_config::MAX_PITCH_ROLL_ACC, input.roll_acc_sp));
  float pitchAccSp = std::max(-unit_config::MAX_PITCH_ROLL_ACC, std::min(unit_config::MAX_PITCH_ROLL_ACC, input.pitch_acc_sp));

  // Constants
  const float M_PI = 3.1415926535;
  const float F_LE   = M_PI * 7/18;    // Fin leading edge angle (rad)
  const float F_TR   = 0;              // Fin trailing edge distance past fin base (m)
  const float F_LEN  = 0.123;          // Fin length (m)
  const float F_AREA = 0.0027552;      // Fin area (m^2)
  const float F_CP   = 0.33 * F_LEN;   // Fin Cp (m)
  const float F_NUM  = 2;              // Number of fins
  const float F_D    = 0.06477;        // Distance from roll axis to fin Cp (m)
  const float I      = 82.58e-6;            // TODO: Rocket rotational inertia

  // Sensor inputs
  float alt = 1414;   // Altitude (m)
  float v_rocket =
    // 10-th degree polynomial approximation of sim 0-11s, apogee around 9s.
    // R2 = 0.9996312149
    - 2.6840582709e-5 * pow(unit_config::launchtime,10)
    + 0.0014233804    * pow(unit_config::launchtime,9)
    - 0.0316956142    * pow(unit_config::launchtime,8)
    + 0.3829791948    * pow(unit_config::launchtime,7)
    - 2.6989140977    * pow(unit_config::launchtime,6)
    + 11.002863196    * pow(unit_config::launchtime,5)
    - 23.739539259    * pow(unit_config::launchtime,4)
    + 22.393657699    * pow(unit_config::launchtime,3)
    - 24.365983223    * pow(unit_config::launchtime,2)
    + 86.00216523     * unit_config::launchtime;

  // TODO: Expand estimate to full 12-space

  // Calculate atmospheric pressure. We should eventually get this from MS5611.
  // Eq. 9 from "A Quick Derivation relating altitude to air pressure", PSAS,
  // 2004.
  float pressure = 100*pow((44331.514-alt)/11880.516, 5.255877);   // Pressure (Pa)
  float temp = 273.15;   // Absolute temperature (K)   TODO: add external thermistor

  // Calculate air density, speed of sound
  float p_air = pressure/(287.058*temp);   // Air density from ideal gas law (kg/m^3)
  float v_air = 20.05*sqrt(temp);   // Speed of sound (m/s)

  // Mach number
  float M = v_rocket / v_air;   // Mach number

  // Mach angle
  //float u = asin(1/M);                   // Unicorn
  //float beta = sqrt(pow(M,2) - 1);       // Bollocks
  //float m = beta * tan(M_PI/2 - F_LE);   // Magic cot(x) = tan(pi/2 - x)

  // Elliptic Integral
  //const float a[] = {0.44325141463,
  //                   0.06260601220,
  //                   0.04757383546,
  //                   0.01736506451};
  //const float b[] = {0.24998368310,
  //                   0.09200180037,
  //                   0.04069697526,
  //                   0.00526449639};
  //float E = 1;
  //for (int i=0; i<4; i++) {
  //  E += a[i]*pow(m,2*(i+1)) + log(1/pow(m,2))*b[i]*pow(m,2*(i+1));   // 0 <= m < 1
  //}

  // Roll controller
  float torque = I * rollAccSp;
  float F_L = torque / (F_NUM * F_D);   // Force required per fin (N)

  // Lift coefficient
  float C_L = 2*F_L/(p_air * pow(v_rocket,2) * F_AREA);   // Subsonic
  //float C_L = (2 * M_PI * m) / (E * beta);   // Supersonic

  // Fin controller
  float rollActuatorSp = 0.5 + C_L/(2*M_PI);   // Fin angle

  // Output
  actuator_setpoint_t setpoint {
    .roll_sp = rollActuatorSp,
    .pitch_sp = 0.0f,
    .yaw_sp = 0.0f,
    .throttle_sp = input.throttle_sp
  };

  return setpoint;
}
