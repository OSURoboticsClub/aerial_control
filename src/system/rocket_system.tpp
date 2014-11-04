template <int num_rotors>
void RocketSystem<num_rotors>::init() {
  getIMU()->init();
}

template <int num_rotors>
void RocketSystem<num_rotors>::update() {
  // Poll the IMU
  imu_reading_t imu_reading = getIMU()->read();

  // Update the attitude estimate
  attitude_estimate_t estimate = getAttitudeEstimator()->update(imu_reading);

  // Poll for controller input
  controller_input_t input = getInputSource()->read();

  // Run the controllers
  angular_position_setpoint_t angular_setpoint = {
    .pitch_pos_sp = input.pitch_sp,
    .roll_pos_sp = input.roll_sp,
    .yaw_pos_sp = input.yaw_sp,
    .throttle_sp = input.throttle_sp
  };

  actuator_setpoint_t actuator_setpoint = runController(estimate, angular_setpoint);

  // Update motor outputs
  getMotorMapper()->run(actuator_setpoint);
}
