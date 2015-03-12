#include "communication/communicator.hpp"

Communicator::Communicator(chibios_rt::BaseSequentialStreamInterface& stream)
  : reader(stream, *this),
    writer(stream) {
}

void Communicator::start() {
  // Start the reader with a slightly higher priority than the writer because
  // reading control messages is more important than writing logs.
  reader.start(LOWPRIO + 1);
  writer.start(LOWPRIO);
}

void Communicator::submit(std::uint8_t b) {
  // TODO: Make `decoded` static?
  protocol::decoded_message_t<255> decoded;
  if(decoder.process(b, &decoded)) {
    dispatch(decoded);
  }
}

void Communicator::registerListener(MessageListener *listener) {
  listeners[listenersPos++] = listener;
}
