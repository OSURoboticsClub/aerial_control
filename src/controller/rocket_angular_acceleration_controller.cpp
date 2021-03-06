#include "controller/rocket_angular_acceleration_controller.hpp"

#include <algorithm>

#include <util/math.hpp>
#include "chprintf.h"

RocketAngularAccelerationController::RocketAngularAccelerationController(ParameterRepository& params)
  : params(params) {
  params.def(PARAM_MAX_PITCH_ROLL_ACC, 0.0);
}

ActuatorSetpoint RocketAngularAccelerationController::run(const WorldEstimate& est, const AngularAccelerationSetpoint& input) {
  // Limit to maximum angular accelerations
  float yawAccSp = input.yawAcc;

  // Constants
  const float F_LE   = M_PI * 7/18;    // Fin leading edge angle (rad)
  const float F_TR   = 0;              // Fin trailing edge distance past fin base (m)
  const float F_LEN  = 0.123;          // Fin length (m)
  const float F_AREA = 0.003075;       // Fin area (m^2)
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
  float torque = I * yawAccSp;
  float F_L = torque / (F_NUM * F_D);   // Force required per fin (N)

  // Lift coefficient
  v_rocket = std::max(60.0f, v_rocket);   // Prevent ridiculously large C_L
  float C_L = 2*F_L/(p_air * pow(v_rocket,2) * F_AREA);   // Subsonic
  //float C_L = (2 * M_PI * m) / (E * beta);   // Supersonic

  // Angle of attack (limit to 10 deg stall angle) based on lift slope of thin
  // airfoil being approximately 2pi per radian
  float alpha = C_L / (2*M_PI);   // Radians
  alpha = std::max(-M_PI/18, std::min(M_PI/18, alpha));   // [-pi/18, pi/18]

  // PWM duty cycle offset
  // Scale factor was obtained by measuring fin angle at 0.5 duty cycle away
  // from center. I.e., 0.5 duty cycle change corresponds to 9.7 degrees of fin
  // movement. Small angle approximation.
  float dc_offset = alpha * 0.5/(9.7*M_PI/180);   // [-0.5, 0.5]

  // Fin controller
  float rollActuatorSp = 0.5 + dc_offset;

  // Output
  ActuatorSetpoint setpoint {
    .roll = 0.0f,
    .pitch = 0.0f,
    .yaw = rollActuatorSp,
    .throttle = input.throttle
  };

  return setpoint;
}
