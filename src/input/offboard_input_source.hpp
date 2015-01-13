#ifndef OFFBOARD_INPUT_SOURCE_HPP_
#define OFFBOARD_INPUT_SOURCE_HPP_

#include "protocol/messages.hpp"

#include "communication/communicator.hpp"
#include "communication/message_listener.hpp"
#include "input/input_source.hpp"

class OffboardInputSource : public InputSource, public MessageListener {
public:
  OffboardInputSource(Communicator& communicator);

  controller_input_t read() override;

  void on(const protocol::message::offboard_attitude_control_message_t& m) override;

private:
  controller_input_t lastInput;
  // TODO: systime_t lastInputTimestamp;
};

#endif
