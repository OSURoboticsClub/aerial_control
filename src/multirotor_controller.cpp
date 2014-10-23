#include <multirotor_controller.hpp>

MultirotorController::MultirotorController(Accelerometer *accel, Gyroscope *gyro,
    AttitudeEstimator *estimator, InputSource *inputSource,
    ControllerPipeline *pipeline, Motor *motors)
  : accel(accel), gyro(gyro),
    estimator(estimator),
    inputSource(inputSource),
    pipeline(pipeline),
    motors(motors) {
}

void MultirotorController::init() {
  // Initialize sensors
  gyro->init();
  accel->init();
}

void MultirotorController::update() {
  // Poll the accelerometer and gyroscope
  struct gyroscope_reading_t gyro_reading = gyro->read();
  struct accelerometer_reading_t accel_reading = accel->read();

  // Update the attitude estimate
  struct attitude_estimate_t estimate = estimator->update(accel_reading, gyro_reading);

  // Poll for controller input
  struct controller_input_t input = inputSource->read();

  // Run the controllers
  struct controller_output_t controller_input = {
    .setpoints = {
      input.roll_sp,
      input.pitch_sp,
      input.yaw_sp,
      input.thrust_sp
    }
  };

  struct controller_output_t controller_output = pipeline->run(estimate, controller_input);

  // Update motor outputs
  struct motor_output_t motor_output = motorMapper.run(controller_output);
  for(int i = 0; i < NUM_ROTORS; i++) {
    // TODO: PWM scaling (0..1)
    // motors[i].setDutyCycle(motor_output.outputs[i]);
  }
}
