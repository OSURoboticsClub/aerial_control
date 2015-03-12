#ifndef TRI_MULTIROTOR_MOTOR_MAPPER_HPP_
#define TRI_MULTIROTOR_MOTOR_MAPPER_HPP_

#include "communication/communicator.hpp"
#include "communication/rate_limited_stream.hpp"
#include "controller/setpoint_types.hpp"
#include "motor/motor_mapper.hpp"
#include "motor/pwm_device_group.hpp"

class MultirotorTriMotorMapper : public MotorMapper {
public:
  MultirotorTriMotorMapper(PWMDeviceGroup<3>& motors, PWMDeviceGroup<1>& servos, Communicator& communicator);

  void run(bool armed, ActuatorSetpoint& input) override;

private:
  PWMDeviceGroup<3> motors;
  PWMDeviceGroup<1> servos;
  RateLimitedStream throttleStream;
};

#endif
