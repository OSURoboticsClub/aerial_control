#include <controller/pid_controller.hpp>

PIDController::PIDController(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd),
      currError(0.0), prevError(0.0), accumError(0.0) {
}

float PIDController::calculate(float sp, float pv, float dt) {
  prevError = currError;
  currError = sp - pv;
  accumError += currError;

  float p = kp * currError;
  float i = ki * accumError;
  float d = kd * (currError - prevError) / dt;

  return p + i + d;
}

