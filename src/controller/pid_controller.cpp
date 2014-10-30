#include <controller/pid_controller.hpp>

PIDController::PIDController(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd),
      currError(0.0), prevError(0.0), accumError(0.0) {
}

float PIDController::calculate(float sp, float pv, float dt) {
  this->prevError = this->currError;
  this->currError = sp - pv;
  this->accumError += currError;

  float p = this->kp * this->currError;
  float i = this->ki * this->accumError;
  float d = this->kd * (this->currError - this->prevError) / dt;

  return p + i + d;
}

