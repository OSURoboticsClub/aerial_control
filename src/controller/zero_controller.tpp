template <typename S>
ZeroController<S>::ZeroController() {
}

template <typename S>
ActuatorSetpoint ZeroController<S>::run(const WorldEstimate& world, const S& input) {
  ActuatorSetpoint setpoint {
    .roll = 0.0f,
    .pitch = 0.0f,
    .yaw = 0.0f,
    .throttle = 0.0f
  };

  return setpoint;
}
