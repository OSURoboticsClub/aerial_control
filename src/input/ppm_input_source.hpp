#ifndef PPM_INPUT_SOURCE_H_
#define PPM_INPUT_SOURCE_H_

#include <array>

#include <hal.h>

#include <input/input_source.hpp>

static constexpr int MODE_MANUAL = 0;
static constexpr int MODE_ALTCTL = 1;
static constexpr int MODE_AUTO   = 2;

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

  static void trigger(ICUDriver *icup);
  ControllerInput read() override;
};

#endif
