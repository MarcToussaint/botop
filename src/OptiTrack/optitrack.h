#pragma once

#include <Kin/kin.h>

namespace libmotioncapture{ class MotionCapture; }

namespace rai{

struct OptiTrack {
  libmotioncapture::MotionCapture *mocap = 0;

  OptiTrack();
  ~OptiTrack();

  void step(rai::Configuration& C);
};

} //namespace
