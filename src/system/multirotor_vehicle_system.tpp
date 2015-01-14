template <typename SP>
actuator_setpoint_t MultirotorVehicleSystem::runPipeline(const attitude_estimate_t& estimate, const SP& sp) {
  actuator_setpoint_t actuatorSp = runController(estimate, sp);
  return actuatorSp;
}

