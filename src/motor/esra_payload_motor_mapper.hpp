#ifndef ESRA_PAYLOAD_MOTOR_MAPPER_HPP_
#define ESRA_PAYLOAD_MOTOR_MAPPER_HPP_

#include "communication/communicator.hpp"
#include "communication/rate_limited_stream.hpp"
#include "controller/setpoint_types.hpp"
#include "motor/motor_mapper.hpp"
#include "motor/pwm_device_group.hpp"

class EsraPayloadMotorMapper : public MotorMapper {
public:
  EsraPayloadMotorMapper(PWMDeviceGroup<1>& motors, Communicator& communicator);

  void run(bool armed, ActuatorSetpoint& input) override;

private:
  PWMDeviceGroup<1> motors;
  RateLimitedStream throttleStream;
};

#endif
