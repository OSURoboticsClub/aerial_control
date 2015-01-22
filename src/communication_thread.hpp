#ifndef COMMUNICATION_THREAD_HPP_
#define COMMUNICATION_THREAD_HPP_

#include "ch.hpp"

#include "communication/communicator.hpp"

class CommunicationThread : public chibios_rt::BaseStaticThread<256> {
public:
  CommunicationThread(Communicator& communicator);

  msg_t main() override;

private:
  Communicator& communicator;
};

#endif
