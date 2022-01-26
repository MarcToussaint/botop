#pragma once

#include <Kin/kin.h>
#include <Core/thread.h>

namespace libmotioncapture{ class MotionCapture; }

namespace rai{

struct OptiTrack : Thread {
  libmotioncapture::MotionCapture *mocap = 0;

  OptiTrack();
  ~OptiTrack();

  void pull(rai::Configuration& C);

  void step();

private:
  std::mutex mux;
  struct Entry{ rai::Frame *frame=0; rai::Transformation pose=0; };
  std::map<const char*, Entry> poses;
};

} //namespace
