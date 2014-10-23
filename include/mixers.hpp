#ifndef MIXERS_HPP_
#define MIXERS_HPP_

static const float MIXER_QUAD_X[][4] = {
  // Roll  Pitch    Yaw
  {  1.0f,  1.0f,  1.0f }, // Front left
  { -1.0f,  1.0f, -1.0f }, // Front right
  { -1.0f, -1.0f,  1.0f }, // Back right
  {  1.0f, -0.0f, -1.0f }, // Back left
};

#endif
