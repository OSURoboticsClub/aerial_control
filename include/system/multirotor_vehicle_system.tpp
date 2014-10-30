template <int num_rotors>
void MultirotorVehicleSystem<num_rotors>::init() {
}

template <int num_rotors>
void MultirotorVehicleSystem<num_rotors>::update() {
  // Poll the accelerometer and gyroscope
  accelerometer_reading_t accel_reading = getAccelerometer()->read();
  gyroscope_reading_t gyro_reading = getGyroscope()->read();

  // Update the attitude estimate
  attitude_estimate_t estimate = getAttitudeEstimator()->update(accel_reading, gyro_reading);

  // Poll for controller input
  controller_input_t input = getInputSource()->read();

  // Run the controllers
  attitude_position_setpoint_t attitude_setpoint = {
    .pitch_pos_sp = input.pitch_sp,
    .roll_pos_sp = input.roll_sp,
    .yaw_pos_sp = input.yaw_sp,
    .throttle_sp = input.throttle_sp
  };

  // actuator_setpoint_t controller_output = getPipeline()->run(estimate, controller_input);
  actuator_setpoint_t setpoint = runController(estimate, attitude_setpoint);

  // TODO: Update motor outputs
  // getMotorMapper()->run(controller_output);
}
