#include "communication/communication_thread.hpp"

CommunicationThread::CommunicationThread(Communicator& communicator)
  : communicator(communicator) {
}

msg_t CommunicationThread::main() {
  while(true) {
    std::uint8_t b = communicator.getStream().get();
    communicator.submit(b);
  }
}
