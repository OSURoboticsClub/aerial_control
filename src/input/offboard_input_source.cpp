#include "input/offboard_input_source.hpp"

OffboardInputSource::OffboardInputSource(Communicator& communicator)
  : MessageListener(communicator) {
}

ControllerInput OffboardInputSource::read() {
  // Invalidate the last input if it was received a long time ago.
  if(lastInputTimestamp + MS2ST(100) < chibios_rt::System::getTime()) {
    lastInput.valid = false;
  } else {
    lastInput.valid = true;
  }

  return lastInput;
}

void OffboardInputSource::on(const protocol::message::offboard_attitude_control_message_t& m) {
  lastInputTimestamp = chibios_rt::System::getTime();
  lastInput = ControllerInput {
    .valid = true,
    .roll = m.roll,
    .pitch = m.pitch,
    .yaw = m.yaw,
    .throttle = m.throttle
  };
}
