template <typename S>
ZeroController<S>::ZeroController() {
}

template <typename S>
actuator_setpoint_t ZeroController<S>::run(const attitude_estimate_t& estimate, const S& input) {
  actuator_setpoint_t setpoint {
    .roll_sp = 0.0f,
    .pitch_sp = 0.0f,
    .yaw_sp = 0.0f,
    .throttle_sp = 0.0f
  };

  return setpoint;
}
