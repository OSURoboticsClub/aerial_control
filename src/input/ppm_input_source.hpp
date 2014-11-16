#ifndef PPM_INPUT_SOURCE_H_
#define PPM_INPUT_SOURCE_H_

#include <array>

#include <hal.h>

#include <input/input_source.hpp>

enum class PPMState {
  /**
   * No start frame found.
   */
  UNSYNCED,

  /**
   * Start frame found. Reading data frames.
   */
  SYNCED
};

class PPMInputSource : public InputSource {
public:
  PPMInputSource();

  void trigger();
  controller_input_t read() override;

private:
  constexpr static int MAX_CHANNELS = 12;
  constexpr static int MIN_START_WIDTH = 2500;
  constexpr static int MIN_CHANNEL_WIDTH = 800;
  constexpr static int MAX_CHANNEL_WIDTH = 2200;

  /**
   * The current decoder state.
   */
  PPMState state;

  /**
   * The current channel.
   */
  int currentChannel;

  /**
   * The time of the last trigger.
   */
  systime_t lastFrameStart;

  std::array<unsigned int, MAX_CHANNELS> channelBuffer;
  std::array<unsigned int, MAX_CHANNELS> channelTempBuffer;
};

#endif
