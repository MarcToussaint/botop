#pragma once

#include <Core/array.h>

struct ExplainBackground {
  //parameters
  int verbose=1;
  float threshold=.02;
  float farThreshold=1.1;
  //filter states
  floatA background;
  byteA countDeeper;
  floatA valueDeeper;

  bool computeBackground = true;

  void compute(byteA& pixelLabels,
               const byteA& cam_color, const floatA& cam_depth);

  void saveBackgroundModel(const char* name = "backgroundModel");
  void loadBackgroundModel(const char* name = "backgroundModel");
};
