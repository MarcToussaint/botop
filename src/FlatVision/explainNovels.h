#pragma once

#include <Core/array.h>
#include "helpers.h"

struct ExplainNovelPercepts {
  //parameters
  int verbose=1;
  //internal
  byteA countUnexplained;
  //output: an array of novel flat percepts
  rai::Array<FlatPercept> flats;

  void compute(byteA& pixelLabels,
               const byteA& cam_color, const floatA& cam_depth);
};
