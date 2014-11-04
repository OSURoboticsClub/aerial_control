#ifndef PWM_MOTOR_MAPPER_HPP_
#define PWM_MOTOR_MAPPER_HPP_

#include <array>

#include <hal.h>
#include <controller/setpoint_types.hpp>
#include <motor/motor_mapper.hpp>

template <int motor_count>
class PWMMotorMapper : public MotorMapper {
public:
  void init() override;
  virtual void run(actuator_setpoint_t& input) =0;

protected:
  PWMMotorMapper();

  void setMotorSpeeds(const std::array<float, motor_count>& percents);

private:
  static void mapToBounds(const std::array<float, motor_count>& percents, std::array<float, motor_count>* mapped);
};

#include <motor/pwm_motor_mapper.tpp>

#endif
