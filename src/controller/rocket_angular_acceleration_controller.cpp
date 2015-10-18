#include "controller/rocket_angular_acceleration_controller.hpp"

#include <algorithm>

#include "unit_config.hpp"
#include "chprintf.h"

RocketAngularAccelerationController::RocketAngularAccelerationController()
  : rollAccPid(unit_config::ANGACC_X_KP, unit_config::ANGACC_X_KI, unit_config::ANGACC_X_KD),
    pitchAccPid(unit_config::ANGACC_Y_KP, unit_config::ANGACC_Y_KI, unit_config::ANGACC_Y_KD),
    yawAccPid(unit_config::ANGACC_Z_KP, unit_config::ANGACC_Z_KI, unit_config::ANGACC_Z_KD) {
}

ActuatorSetpoint RocketAngularAccelerationController::run(const WorldEstimate& est, const AngularAccelerationSetpoint& input) {
  // Limit to maximum angular accelerations
  float rollAccSp = std::max(-unit_config::MAX_PITCH_ROLL_ACC, std::min(unit_config::MAX_PITCH_ROLL_ACC, input.rollAcc));

  // Constants
  const float M_PI   = 3.1415926535;
  const float F_LE   = M_PI * 7/18;    // Fin leading edge angle (rad)
  const float F_TR   = 0;              // Fin trailing edge distance past fin base (m)
  const float F_LEN  = 0.123;          // Fin length (m)
  const float F_AREA = 0.0027552;      // Fin area (m^2)
  const float F_CP   = 0.33 * F_LEN;   // Fin Cp (m)
  const float F_NUM  = 2;              // Number of fins
  const float F_D    = 0.06477;        // Distance from roll axis to fin Cp (m)
  const float I      = 1.45;           // Rocket rotational inertia (kg*m^2)

  // Sensor inputs
  //float alt = est.loc.alt;   // Altitude (m)
  float v_rocket = est.loc.dAlt;   // Vertical velocity (m/s)   TODO(yoos): no position data, so we assume lateral speed is negligible.
  // TODO: Expand estimate to full 12-space
  float pressure = 100*(*est.sensors.bar).pressure;   // Pressure (Pa)
  float temp = 273.15 + (*est.sensors.bar).temperature;   // Absolute temperature (K)   TODO: add external thermistor

  // DEBUG
  //static int i=0;
  //if (i==0) {
  //  BaseSequentialStream* chp = (BaseSequentialStream*)&SD4;
  //  chprintf(chp, "%d: %f %f %f\r\n", chibios_rt::System::getTime(), v_rocket, pressure, temp);
  //}
  //i = (i+1) % 50;

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
  v_rocket = std::max(20.0f, v_rocket);   // Prevent ridiculously large C_L
  float C_L = 2*F_L/(p_air * pow(v_rocket,2) * F_AREA);   // Subsonic
  //float C_L = (2 * M_PI * m) / (E * beta);   // Supersonic

  // Angle of attack (limit to 10 deg stall angle)
  float alpha = C_L * 90/M_PI/M_PI;   // Radians
  alpha = std::max(-M_PI/18, std::min(M_PI/18, alpha));   // [-pi/18, pi/18]

  // PWM duty cycle offset
  // Scale factor was obtained by measuring fin angle at 0.1 duty cycle away
  // from center.
  float dc_offset = alpha * 0.1 / 0.422854;   // [-0.5, 0.5]

  // Fin controller
  float rollActuatorSp = 0.5 + dc_offset;

  // Output
  ActuatorSetpoint setpoint {
    .roll = rollActuatorSp,
    .pitch = 0.0f,
    .yaw = 0.0f,
    .throttle = input.throttle
  };

  return setpoint;
}
