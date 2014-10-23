#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

class PIDController {
public:
  PIDController(float kp, float ki, float kd);

  float calculate(float sp, float pv, float dt);

private:
  float kp;
  float ki;
  float kd;

  float curr_error;
  float prev_error;
  float accum_error;
};

#endif
