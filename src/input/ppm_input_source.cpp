#include <input/ppm_input_source.hpp>

#include <util/time.hpp>
#include <chprintf.h>

static constexpr int MAX_CHANNELS = 12;
static constexpr int MIN_START_WIDTH = 2500;
static constexpr int MIN_CHANNEL_WIDTH = 1106;   // 1116 - 10
static constexpr int MAX_CHANNEL_WIDTH = 1926;   // 1916 + 10
static constexpr int MID_CHANNEL_WIDTH = (MIN_CHANNEL_WIDTH+MAX_CHANNEL_WIDTH)/2;
       
static constexpr int CHANNEL_YAW      = 0;
static constexpr int CHANNEL_PITCH    = 1;
static constexpr int CHANNEL_THROTTLE = 2;
static constexpr int CHANNEL_ROLL     = 3;
static constexpr int CHANNEL_ARM      = 4;
static constexpr int CHANNEL_VELMODE  = 5;
static constexpr int CHANNEL_UNUSED   = 6;
static constexpr int CHANNEL_MODE     = 7;

/**
 * The current decoder state.
 */
static PPMState state;

/**
 * The current channel.
 */
static int currentChannel;

/**
 * The time of the last trigger.
 */
static systime_t lastPulseStart;

static std::array<unsigned int, MAX_CHANNELS> channelBuffer;
static std::array<unsigned int, MAX_CHANNELS> channelTempBuffer;

static const ICUConfig ICUD2_CONFIG = {
  ICU_INPUT_ACTIVE_HIGH,
  200000,   // 200 kHz ICU clock frequency
  NULL,
  PPMInputSource::trigger,
  NULL,
  ICU_CHANNEL_1,
  0
};

PPMInputSource::PPMInputSource() {
  state = PPMState::UNSYNCED;
  currentChannel = 0;
  lastPulseStart = 0;
  channelBuffer[CHANNEL_ROLL]     = MID_CHANNEL_WIDTH;
  channelBuffer[CHANNEL_PITCH]    = MID_CHANNEL_WIDTH;
  channelBuffer[CHANNEL_YAW]      = MID_CHANNEL_WIDTH;
  channelBuffer[CHANNEL_THROTTLE] = MIN_CHANNEL_WIDTH;
  channelBuffer[CHANNEL_MODE]     = MAX_CHANNEL_WIDTH;
  channelBuffer[CHANNEL_VELMODE]  = MIN_CHANNEL_WIDTH;
  channelBuffer[CHANNEL_ARM]      = MAX_CHANNEL_WIDTH;

  icuInit();
  icuStart(&ICUD2, &ICUD2_CONFIG); // TODO
  palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_PULLDOWN | PAL_MODE_ALTERNATE(1));
  icuEnable(&ICUD2);
}

void PPMInputSource::trigger(ICUDriver *icup) {
  // TODO: `chTimeNow` can wrap around. Need to handle this case.
  systime_t pulseStart = chTimeNow();

  uint32_t lastPulseWidth = ST2US(pulseStart - lastPulseStart);
  switch (state) {
    case PPMState::UNSYNCED:
      // If this frame looks like a start frame, ignore it and move to the
      // LOCKED state.
      if (lastPulseWidth > MIN_START_WIDTH) {
        state = PPMState::SYNCED;
      }

      break;
    case PPMState::SYNCED:
      if (lastPulseWidth > MIN_START_WIDTH) {
        // Start frame. Reset channel counter and flip the buffer.
        currentChannel = 0;
        channelBuffer = channelTempBuffer; // copy

        state = PPMState::SYNCED;
      }
      else if (lastPulseWidth > MIN_CHANNEL_WIDTH &&
                 lastPulseWidth < MAX_CHANNEL_WIDTH) {
        // Channel frame. Only store if we have room.
        if (currentChannel < MAX_CHANNELS) {
          channelTempBuffer[currentChannel] = lastPulseWidth;
          currentChannel++;
        }
      }
      else {
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
  ControllerInput input = {
    .valid = (state == PPMState::SYNCED),
    .roll     = ((float) channelBuffer[CHANNEL_ROLL]     - MIN_CHANNEL_WIDTH) / (MAX_CHANNEL_WIDTH-MIN_CHANNEL_WIDTH) * 2 - 1,
    .pitch    = ((float) channelBuffer[CHANNEL_PITCH]    - MIN_CHANNEL_WIDTH) / (MAX_CHANNEL_WIDTH-MIN_CHANNEL_WIDTH) * 2 - 1,
    .yaw      = ((float) channelBuffer[CHANNEL_YAW]      - MIN_CHANNEL_WIDTH) / (MAX_CHANNEL_WIDTH-MIN_CHANNEL_WIDTH) * 2 - 1,
    .throttle = ((float) channelBuffer[CHANNEL_THROTTLE] - MIN_CHANNEL_WIDTH) / (MAX_CHANNEL_WIDTH-MIN_CHANNEL_WIDTH),
    .mode = (channelBuffer[CHANNEL_MODE] > MID_CHANNEL_WIDTH) ? 0 : 1,
    .velocityMode = (channelBuffer[CHANNEL_VELMODE] > MID_CHANNEL_WIDTH),
    .armed = (channelBuffer[CHANNEL_ARM] < MID_CHANNEL_WIDTH)
  };

  // Flip yaw
  input.yaw *= -1;

  // DEBUG
  //static int loop=0;
  //if (loop == 0) {
  //  chprintf((BaseSequentialStream*)&SD4, "CI %f %f %f %f %d %d %d\r\n", input.roll, input.pitch, input.yaw, input.throttle, input.mode, input.velocityMode, input.armed);
  //}
  //loop = (loop+1) % 50;

  return input;
}
