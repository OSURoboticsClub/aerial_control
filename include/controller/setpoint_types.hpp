#ifndef SETPOINT_TYPES_HPP_
#define SETPOINT_TYPES_HPP_

struct attitude_setpoint_t {
  float pitch_pos_sp;
  float roll_pos_sp;
  float yaw_pos_sp;
};

struct attitude_velocity_setpoint_t {
  float pitch_vel_sp;
  float roll_vel_sp;
  float yaw_vel_sp;
};

struct actuator_setpoint_t {
  float pitch_sp;
  float roll_sp;
  float yaw_sp;
};

#endif
