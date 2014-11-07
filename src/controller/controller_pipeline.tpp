template <typename R> template <typename SP, typename C>
R ControllerPipeline<R>::run(const attitude_estimate_t& estimate, const SP& input, C& tail) {
  return tail.run(estimate, input);
}

template <typename R> template <typename SP, typename C, typename... Cs>
R ControllerPipeline<R>::run(const attitude_estimate_t& estimate, const SP& input, C& head, Cs&... controllers) {
  auto result = head.run(estimate, input);

  return run(estimate, result, controllers...);
}
