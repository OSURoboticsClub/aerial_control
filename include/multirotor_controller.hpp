#ifndef MULTIROTOR_CONTROLLER_HPP_
#define MULTIROTOR_CONTROLLER_HPP_

#include <config.hpp>
#include <motor_mapper.hpp>
#include <motor.hpp>
#include <controller/attitude_controller.hpp>
#include <controller/attitude_rate_controller.hpp>
#include <controller/controller_pipeline.hpp>
#include <estimator/attitude_estimator.hpp>
#include <input/input_source.hpp>
#include <sensor/accelerometer.hpp>
#include <sensor/gyroscope.hpp>

class MultirotorController {
public:
  MultirotorController(Accelerometer *accel, Gyroscope *gyro,
                        AttitudeEstimator *estimator, InputSource *inputSource,
                        ControllerPipeline *pipeline,
                        Motor *motors);
  void init();
  void update();

private:
  Accelerometer *accel;
  Gyroscope *gyro;

  AttitudeEstimator *estimator;
  InputSource *inputSource;

  ControllerPipeline *pipeline;

  MotorMapper motorMapper;
  Motor *motors;
};

#endif
