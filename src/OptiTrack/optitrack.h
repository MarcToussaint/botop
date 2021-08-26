#pragma once

#include <Kin/kin.h>

namespace libmotioncapture{ class MotionCapture; }

struct OptiTrack {
  rai::Configuration& C;
  libmotioncapture::MotionCapture *mocap = 0;

  OptiTrack(rai::Configuration& _C);
  ~OptiTrack();

  void step();
};
