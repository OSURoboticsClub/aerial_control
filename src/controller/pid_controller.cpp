#include <controller/pid_controller.hpp>

PIDController::PIDController(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd),
      curr_error(0.0), prev_error(0.0), accum_error(0.0) {
}

float PIDController::calculate(float sp, float pv, float dt) {
  this->prev_error = this->curr_error;
  this->curr_error = sp - pv;
  this->accum_error += curr_error;

  float p = this->kp * this->curr_error;
  float i = this->ki * this->accum_error;
  float d = this->kd * (this->curr_error - this->prev_error) / dt;

  return p + i + d;
}

