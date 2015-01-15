#include "input/offboard_input_source.hpp"

OffboardInputSource::OffboardInputSource(Communicator& communicator)
  : MessageListener(communicator) {
}

controller_input_t OffboardInputSource::read() {
  return lastInput;
}

void OffboardInputSource::on(const protocol::message::offboard_attitude_control_message_t& m) {
  lastInput = controller_input_t {
    .roll_sp = m.roll,
    .pitch_sp = m.pitch,
    .yaw_sp = m.yaw,
    .throttle_sp = m.throttle
  };
}
