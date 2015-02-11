#ifndef QUAD_X_MULTIROTOR_MOTOR_MAPPER_HPP_
#define QUAD_X_MULTIROTOR_MOTOR_MAPPER_HPP_

#include "communication/communicator.hpp"
#include "communication/rate_limited_stream.hpp"
#include "controller/setpoint_types.hpp"
#include "motor/motor_mapper.hpp"
#include "motor/pwm_device_group.hpp"

class MultirotorQuadXMotorMapper : public MotorMapper {
public:
  MultirotorQuadXMotorMapper(PWMDeviceGroup<4>& motors, Communicator& communicator);

  void run(bool armed, ActuatorSetpoint& input) override;

private:
  PWMDeviceGroup<4> motors;
  RateLimitedStream throttleStream;
};

#endif
