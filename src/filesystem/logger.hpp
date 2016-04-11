#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include <array>

#include "ch.hpp"
#include "hal.h"
#include "protocol/protocol.hpp"

#include "communication/rate_limited_stream.hpp"
#include "filesystem/filesystem.hpp"
#include "filesystem/ringbuffer.hpp"
//#include "variable_registry/subscriber.hpp"
#include "variant/platform.hpp"

#if USE_FILESYSTEM==TRUE

//class Logger : public VariableRegistrySubscriber, public chibios_rt::BaseStaticThread<1024>{
class Logger : public chibios_rt::BaseStaticThread<1024> {
public:
  Logger(Platform& platform);

  msg_t main(void) override;

  /**
   * Write to filesystem.
   */
  template <typename M>
  void write(const M& message);

private:
  Platform& platform;
  FileSystem fs;
  bool fsReady;

  rb_t buf;   // Ringbuffer
  std::uint8_t _buf[60000];
  std::uint8_t writebuf[8000];

  protocol::Encoder encoder;
};

#include "logger.tpp"

#endif // USE_FILESYSTEM==TRUE

#endif
