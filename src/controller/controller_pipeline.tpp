template <typename R> template <typename SP, typename C>
R ControllerPipeline<R>::run(const WorldEstimate& world, const SP& input, C& tail) {
  return tail.run(world, input);
}

template <typename R> template <typename SP, typename C, typename... Cs>
R ControllerPipeline<R>::run(const WorldEstimate& world, const SP& input, C& head, Cs&... controllers) {
  auto result = head.run(world, input);

  return run(world, result, controllers...);
}
