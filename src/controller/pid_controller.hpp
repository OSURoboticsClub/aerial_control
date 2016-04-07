#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

/**
 * A proportional-integral-derivative controller helper.
 */
class PIDController {
public:
  PIDController();
  PIDController(float kp, float ki, float kd);

  float calculate(float sp, float pv, float dt);

  void setGains(float kp, float ki, float kd);

private:
  float kp;
  float ki;
  float kd;

  float currError;
  float prevError;
  float accumError;
};

#endif
