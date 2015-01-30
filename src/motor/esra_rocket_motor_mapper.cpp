#include "motor/esra_rocket_motor_mapper.hpp"

#include <array>
#include <cstddef>
#include <cmath>

EsraRocketMotorMapper::EsraRocketMotorMapper(PWMDeviceGroup<1>& servos, Communicator& communicator)
  : servos(servos),
    throttleStream(communicator, 10) {
}

void EsraRocketMotorMapper::run(bool armed, actuator_setpoint_t& input) {
  // Constants
  const float M_PI = 3.1415926535;
  const float F_LE   = M_PI * 7/18;    // Fin leading edge angle (rad)
  const float F_TR   = 0;              // Fin trailing edge distance past fin base (m)
  const float F_LEN  = 0.123;          // Fin length (m)
  const float F_AREA = 0.0027552;      // Fin area (m^2)
  const float F_CP   = 0.33 * F_LEN;   // Fin Cp (m)
  const float F_NUM  = 2;              // Number of fins
  const float F_D    = 0.06477;        // Distance from roll axis to fin Cp (m)

  // Sensor inputs
  float alt = 4900;   // Altitude (m)
  float v_rocket = 0;   // TODO: Magically figure this out from sensors (m/s)

  // Calculate atmospheric pressure. We should eventually get this from MS5611.
  // Eq. 9 from "A Quick Derivation relating altitude to air pressure", PSAS,
  // 2004.
  float pressure = 100*pow((44331.514-alt)/11880.516, 5.255877);   // Pressure (Pa)
  float temp = 273.15;   // Absolute temperature (K)   TODO: add external thermistor

  // Calculate air density, speed of sound
  float p_air = pressure/(287.058*temp);   // Air density from ideal gas law (kg/m^3)
  float v_air = 20.05*sqrt(temp+273.15);   // Speed of sound (m/s)

  // Mach number
  float M = v_rocket / v_air;   // Mach number

  // Mach angle
  float u = asin(1/M);                   // Unicorn
  float beta = sqrt(pow(M,2) - 1);       // Bollocks
  float m = beta * tan(M_PI/2 - F_LE);   // Magic cot(x) = tan(pi/2 - x)

  // Elliptic Integral
  const float a[] = {0.44325141463,
                     0.06260601220,
                     0.04757383546,
                     0.01736506451};
  const float b[] = {0.24998368310,
                     0.09200180037,
                     0.04069697526,
                     0.00526449639};
  float E = 1;
  for (int i=0; i<4; i++) {
    E += a[i]*pow(m,2*(i+1)) + log(1/pow(m,2))*b[i]*pow(m,2*(i+1));   // 0 <= m < 1
  }

  // Lift coefficient
  float C_L = (2 * M_PI * m) / (E * beta);

  // Roll controller
  float torque = input.roll_sp;   // TODO: properly implement PID
  float F_L = torque / (F_NUM * F_D);   // Force required per fin (N)

  // Fin controller
  float fin_angle = (2*F_L) / (C_L * p_air * pow(v_air,2) * F_AREA);   // Output fin angle

  // Output
  std::array<float, 1> outputs { fin_angle };
  servos.set(armed, outputs);
}
