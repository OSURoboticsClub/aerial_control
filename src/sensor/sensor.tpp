
template <int axes_count>
Sensor<axes_count>::Sensor() {
  // Default to device axes and zero offsets.
  for (std::size_t i = 0; i < axes_count; i++) {
    axes[i] = i;      // By default, axis 0 -> 0, 1 -> 1, etc.,
    signs[i] = 1;     // all signs are positive,
    offsets[i] = 0.0; // and all offsets are zero.
  }
}

template <int axes_count>
void Sensor<axes_count>::setAxisConfig(
    std::array<int, axes_count> newAxisConfig) {
  for (std::size_t i = 0; i < axes_count; i++) {
    signs[i] = (newAxisConfig[i] > 0) ? 1 : -1;
    axes[i] = signs[i] * newAxisConfig[i] - 1;
  }
}

template <int axes_count>
void Sensor<axes_count>::setOffsets(std::array<float, axes_count> newOffsets) {
  for (std::size_t i = 0; i < axes_count; i++) {
    offsets[i] = newOffsets[i];
  }
}

template <int axes_count>
std::array<float, axes_count> Sensor<axes_count>::getOffsets(void) {
  return offsets;
}
