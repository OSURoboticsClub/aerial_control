template <int num_rotors>
MultirotorVehicleSystem<num_rotors>::MultirotorVehicleSystem(Communicator& communicator)
  : VehicleSystem(communicator) {
}

template <int num_rotors>
void MultirotorVehicleSystem<num_rotors>::init() {
  // TODO: For now, arm the vehicle on initialization. In the future, this
  // should be removed and arming should be designated by a control input or
  // from a communication link.
  setArmed(true);
}

template <int num_rotors>
void MultirotorVehicleSystem<num_rotors>::update() {
  // Poll the gyroscope and accelerometer
  gyroscope_reading_t gyro_reading = getGyroscope().readGyro();
  accelerometer_reading_t accel_reading = getAccelerometer().readAccel();

  // Update the attitude estimate
  attitude_estimate_t estimate = getAttitudeEstimator().update(gyro_reading, accel_reading);

  // Poll for controller input
  controller_input_t input = getInputSource().read();

  // Run the controllers
  angular_position_setpoint_t angular_setpoint {
    .roll_pos_sp = input.roll_sp,
    .pitch_pos_sp = input.pitch_sp,
    .yaw_pos_sp = input.yaw_sp,
    .throttle_sp = input.throttle_sp
  };

  actuator_setpoint_t actuator_setpoint;
  if(isArmed()) {
    // Run the controller pipeline as determined by the subclass
    actuator_setpoint = runController(estimate, angular_setpoint);
  } else {
    // Run the zero controller
    actuator_setpoint = zeroController.run(estimate, angular_setpoint);
  }

  // Update motor outputs
  getMotorMapper().run(actuator_setpoint);
}
