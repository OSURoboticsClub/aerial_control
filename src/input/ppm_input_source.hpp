#ifndef PPM_INPUT_SOURCE_H_
#define PPM_INPUT_SOURCE_H_

#include <array>

#include <hal.h>

#include "input/input_source.hpp"
#include "variant/icu_platform.hpp"

/**
 * The width, in microseconds, of a pulse on a particular channel.
 */
using ChannelWidth = unsigned int;

/**
 * A chanel's index into the internal channel buffer. These are ordered as they
 * arrive in the PPM signal.
 */
using ChannelIndex = unsigned int;

struct PPMInputSourceConfig {
  ChannelWidth minStartWidth;
  ChannelWidth minChannelWidth;
  ChannelWidth maxChannelWidth;

  ChannelIndex channelThrottle;
  ChannelIndex channelRoll;
  ChannelIndex channelPitch;
  ChannelIndex channelYaw;
  ChannelIndex channelArmed;
  ChannelIndex channelVelocityMode;
  ChannelIndex channelRange;
  ChannelIndex channelControlMode;
};

/**
 * Whether a channel's range is -1..1 (full scale) or 0..1 (half scale).
 */
enum class PPMChannelType {
  FULL_SCALE,
  HALF_SCALE
};

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

// TODO: This should probably be configurable
enum class InputControlMode {
  MANUAL,
  ALTCTL,
  AUTO
};

class PPMInputSource : public InputSource, ICUTriggerable {
public:
  PPMInputSource(ICUPlatform& icu, const PPMInputSourceConfig config);

  /**
   * Called to signal the start of a pulse.
   */
  void trigger(ICUDriver *icup);

  ControllerInput read() override;

private:
  /**
   * The maximum number of supported PPM channels. Note that this is constrained
   * by the PPM frame width.
   */
  static const ChannelIndex MAX_CHANNELS = 12;

  /**
   * Scale a channel's raw pulse width to either -1..1 or 0..1.
   */
  float scaleChannel(PPMChannelType type, ChannelWidth input);

  /**
   * Map channel to index values
   *
   * @param divisions Number of indices to chop up the channel into.
   */
  uint8_t channelToIndex(ChannelWidth input, uint8_t divisions);

  const PPMInputSourceConfig config;

  /**
   * The current decoder state.
   */
  PPMState state;

  /**
   * The current channel.
   */
  ChannelIndex currentChannel;

  /**
   * The time of the last trigger.
   */
  systime_t lastPulseStart;

  std::array<ChannelIndex, MAX_CHANNELS> channelBuffer;
  std::array<ChannelIndex, MAX_CHANNELS> channelTempBuffer;
};

#endif
