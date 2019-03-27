#pragma once

#include <Core/array.h>
#include "helpers.h"

struct ExplainRobotPart {
  //parameters
  int verbose=1;
  PixelLabel label=PL_robot;
  //calibration output
  arr calib; //xy-shift (in pixels), z-shift (in meters), xy-tilt (in slope/pixel), z-tilt (in sin(phi))

  void compute(byteA& pixelLabels,
               const byteA& cam_color, const floatA& cam_depth,
               const byteA& model_segments, const floatA& model_depth);
};
