#include "communication/communicator.hpp"

Communicator::Communicator(chibios_rt::BaseSequentialStreamInterface& stream)
  : stream(stream) {
}

void Communicator::submit(std::uint8_t b) {
  // TODO: Make `decoded` static?
  protocol::decoded_message_t<255> decoded;
  if(decoder.process(b, &decoded)) {
    dispatch(decoded);
  }
}

chibios_rt::BaseSequentialStreamInterface& Communicator::getStream() {
  return stream;
}
