#ifndef SINK_HPP_
#define SINK_HPP_

#include <cstdint>

class Sink {
public:
  virtual void submit(std::uint8_t b) = 0;
};

#endif
