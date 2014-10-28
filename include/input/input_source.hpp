#ifndef INPUT_SOURCE_HPP_
#define INPUT_SOURCE_HPP_

// TODO: Think about how to generalize this. Some input sources may provide
// roll/pitch/yaw setpoints, other may provide x_pos/y_pos/z_pos, etc.
struct controller_input_t {
  float roll_sp;
  float pitch_sp;
  float yaw_sp;
  float thrust_sp;
};

class InputSource {
public:
  virtual controller_input_t read() =0;
};

#endif
