#pragma once

#include <Core/array.h>

struct ExplainBackground {
  //parameters
  int verbose=1;
  float threshold=.01;
  float farThreshold=1.1;
  //filter states
  floatA background;
  byteA countDeeper;
  floatA valueDeeper;

  void compute(byteA& pixelLabels,
               const byteA& cam_color, const floatA& cam_depth);
};
