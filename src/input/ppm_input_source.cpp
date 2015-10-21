#ifdef HAL_USE_ICU

#include "input/ppm_input_source.hpp"

#include "util/time.hpp"

PPMInputSource::PPMInputSource(const PPMInputSourceConfig config)
  : config(config), state(PPMState::UNSYNCED),
    currentChannel(0), lastPulseStart(0),
    channelBuffer{0} {

  auto& icu = ICUPlatform::getInstance();
  icu.registerTrigger(this);

  // TODO(yoos): Set defaults in unit
  ChannelWidth midChannelWidth = (config.minChannelWidth + config.maxChannelWidth) / 2;
  channelBuffer[config.channelThrottle]     = config.minChannelWidth;
  channelBuffer[config.channelRoll]         = midChannelWidth;
  channelBuffer[config.channelPitch]        = midChannelWidth;
  channelBuffer[config.channelYaw]          = midChannelWidth;
  channelBuffer[config.channelArmed]        = config.maxChannelWidth;
  channelBuffer[config.channelVelocityMode] = config.minChannelWidth;
  channelBuffer[config.channelRange]        = config.minChannelWidth;
  channelBuffer[config.channelControlMode]  = config.maxChannelWidth;
}

void PPMInputSource::trigger(ICUDriver *icup) {
  // TODO: `chTimeNow` can wrap around. Need to handle this case.
  systime_t pulseStart = chTimeNow();

  uint32_t lastPulseWidth = ST2US(pulseStart - lastPulseStart);
  switch (state) {
    case PPMState::UNSYNCED:
      // If this frame looks like a start frame, ignore it and move to the
      // LOCKED state.
      if (lastPulseWidth > config.minStartWidth) {
        state = PPMState::SYNCED;
      }

      break;
    case PPMState::SYNCED:
      if (lastPulseWidth > config.minStartWidth) {
        // Start frame. Reset channel counter and flip the buffer.
        currentChannel = 0;
        channelBuffer = channelTempBuffer; // copy

        state = PPMState::SYNCED;
      } else if (lastPulseWidth > config.minChannelWidth &&
                 lastPulseWidth < config.maxChannelWidth) {
        // Channel frame. Only store if we have room.
        if (currentChannel < MAX_CHANNELS) {
          channelTempBuffer[currentChannel] = lastPulseWidth;
          currentChannel++;
        }
      } else {
        // Something went wrong. Reset to unsynced state.
        state = PPMState::UNSYNCED;
      }
      break;
    default:
      break;
  }

  lastPulseStart = pulseStart;
}

ControllerInput PPMInputSource::read() {
  // TODO(yoos): mode and velocityMode channels currently amount to magic
  // numbers. Reimplement.
  ControllerInput input = {
    .valid    = (state == PPMState::SYNCED),
    .roll     = scaleChannel(PPMChannelType::FULL_SCALE, channelBuffer[config.channelRoll]),
    .pitch    = scaleChannel(PPMChannelType::FULL_SCALE, channelBuffer[config.channelPitch]),
    .yaw      = scaleChannel(PPMChannelType::FULL_SCALE, channelBuffer[config.channelYaw]),
    .throttle = scaleChannel(PPMChannelType::HALF_SCALE, channelBuffer[config.channelThrottle]),
    .mode     = channelToIndex(channelBuffer[config.channelControlMode], 3),
    .velocityMode = channelToIndex(channelBuffer[config.channelVelocityMode], 2),
    .armed    = (channelToIndex(channelBuffer[config.channelArmed], 2) == 1)
  };

  return input;
}

float PPMInputSource::scaleChannel(PPMChannelType type, ChannelWidth input) {
  // Don't try to scale invalid inputs.
  if(input < config.minChannelWidth || input > config.maxChannelWidth) {
    return 0.0;
  }

  float mid = (config.minChannelWidth + config.maxChannelWidth) / 2.0;
  float width = config.maxChannelWidth - config.minChannelWidth;
  float halfScale = (input - config.minChannelWidth) / width;

  if(type == PPMChannelType::FULL_SCALE) {
    return halfScale * 2.0 - 1.0;
  } else { // type == PPMChannelType::HALF_SCALE
    return halfScale;
  }
}

uint8_t PPMInputSource::channelToIndex(ChannelWidth input, uint8_t divisions) {
  if (divisions < 2) {
    return 0;
  }

  float dcDiv = 1.0 / (divisions-1);
  float zeroMaxDC = dcDiv / 2;
  float inputDC = scaleChannel(PPMChannelType::HALF_SCALE, input);
  uint8_t idx = 0;

  while (inputDC > zeroMaxDC + dcDiv*idx) {
    idx++;
  }

  return idx;
}

#endif // HAL_USE_ICU
