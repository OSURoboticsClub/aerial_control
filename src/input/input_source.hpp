#ifndef INPUT_SOURCE_HPP_
#define INPUT_SOURCE_HPP_

// TODO: Think about how to generalize this. Some input sources may provide
// roll/pitch/yaw setpoints, other may provide x_pos/y_pos/z_pos, etc.
struct ControllerInput {
  bool valid;
  float roll;
  float pitch;
  float yaw;
  float throttle;
  bool armed;
};

/**
 * A source for input setpoints.
 */
class InputSource {
public:
  /**
   * Reads from the source and returns the setpoints.
   */
  virtual ControllerInput read() = 0;
};

#endif
