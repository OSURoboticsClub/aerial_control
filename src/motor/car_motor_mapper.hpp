#ifndef CAR_MOTOR_MAPPER_HPP_
#define CAR_MOTOR_MAPPER_HPP_

#include "communication/communicator.hpp"
#include "communication/rate_limited_stream.hpp"
#include "controller/setpoint_types.hpp"
#include "motor/motor_mapper.hpp"
#include "motor/pwm_device_group.hpp"

/**
 * A motor mapper for a car with four independently rotatable wheels.
 */
class CarMotorMapper : public MotorMapper {
public:
  CarMotorMapper(PWMDeviceGroup<4>& motorDevices, PWMDeviceGroup<4>& servoDevices, Communicator& communicator);

  void run(bool armed, actuator_setpoint_t& input) override;

private:
  PWMDeviceGroup<4> motorDevices;
  PWMDeviceGroup<4> servoDevices;
  RateLimitedStream throttleStream;
};

#endif
