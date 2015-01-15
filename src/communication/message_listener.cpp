#include "communication/message_listener.hpp"

#include "communication/communicator.hpp"

MessageListener::MessageListener(Communicator& communicator) {
  communicator.registerListener(this);
}
