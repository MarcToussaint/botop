#pragma once

#include <Kin/kin.h>
#include <Core/thread.h>

namespace libmotioncapture{ class MotionCapture; }

namespace rai{

struct OptiTrack : Thread {
  libmotioncapture::MotionCapture *mocap = 0;

  RAI_PARAM("optitrack/", double, filter, .9)

  OptiTrack();
  ~OptiTrack();

  void pull(rai::Configuration& C);

  void step();

private:
  std::mutex mux;
  std::map<std::string, rai::Transformation> poses;
};

} //namespace
