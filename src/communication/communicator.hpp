#ifndef COMMUNICATOR_HPP_
#define COMMUNICATOR_HPP_

#include <vector>

#include "ch.hpp"
#include "hal.h"
#include "protocol/protocol.hpp"

#include "communication/message_listener.hpp"
#include "communication/sink.hpp"
#include "communication/reader_thread.hpp"
#include "communication/writer_thread.hpp"

/**
 * Manages a list of message callbacks, calling them whenever the proper message
 * type is received.
 */
// TODO(kyle): Refactor this out into a `MessageCallbackDispatcher` or something?
class Communicator : public Sink {
public:
  Communicator(chibios_rt::BaseSequentialStreamInterface& stream);

  /**
   * Start the reader and writer threads.
   */
  void start();

  /**
   * Feed a single byte of input data.
   */
  void submit(std::uint8_t b) override;

  /**
   * Called when a message of type `M` is received.
   */
  template <typename M>
  void on(const M& m);

  /**
   * Send a message.
   *
   * NOTE: Messages are copied and sent on a separate thread, so this will not
   * block on I/O.
   */
  template <typename M>
  void send(const M& message);

  /**
   * Register a new message listener.
   */
  void registerListener(MessageListener *listener);

private:
  /**
   * For a given decoded message, select and call the correct `on` template
   * specialization.
   */
  template <std::size_t buffer_size>
  void dispatch(const protocol::decoded_message_t<buffer_size>& decoded);

  ReaderThread reader;
  WriterThread writer;

  protocol::Encoder encoder;
  protocol::Decoder decoder;

  std::vector<MessageListener *> listeners;
};

#include "communicator.tpp"

#endif
