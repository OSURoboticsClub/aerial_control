#ifndef MESSAGE_LISTENER_HPP_
#define MESSAGE_LISTENER_HPP_

#include "protocol/messages.hpp"

class Communicator;

class MessageListener {
public:
  // TODO: Any way to do this without explicitly overloading for each message
  // type? Can't use templates because they can't be virtual.
  virtual void on(const protocol::message::heartbeat_message_t& m) {}
  virtual void on(const protocol::message::log_message_t& m) {}
  virtual void on(const protocol::message::attitude_message_t& m) {}
  virtual void on(const protocol::message::set_arm_state_message_t& m) {}
  virtual void on(const protocol::message::set_control_mode_message_t& m) {}
  virtual void on(const protocol::message::offboard_attitude_control_message_t& m) {}

  // TODO(kyle): This causes linker errors
  // protected:
  //   MessageListener();
};

#endif
