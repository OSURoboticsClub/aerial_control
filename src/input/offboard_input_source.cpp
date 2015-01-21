#include "input/offboard_input_source.hpp"

OffboardInputSource::OffboardInputSource(Communicator& communicator)
  : MessageListener(communicator) {
}

controller_input_t OffboardInputSource::read() {
  if(lastInputTimestamp + MS2ST(100) < chibios_rt::System::getTime()) {
    lastInput.valid = false;
  } else {
    lastInput.valid = true;
  }

  return lastInput;
}

void OffboardInputSource::on(const protocol::message::offboard_attitude_control_message_t& m) {
  lastInputTimestamp = chibios_rt::System::getTime();
  lastInput = controller_input_t {
    .valid = true,
    .roll_sp = m.roll,
    .pitch_sp = m.pitch,
    .yaw_sp = m.yaw,
    .throttle_sp = m.throttle
  };
}
