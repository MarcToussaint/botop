#pragma once

#include <Core/array.h>
#include <Core/thread.h>

namespace rs2 { class pipeline; }

struct RealSenseThread : Thread {
  struct sRealSenseThread *s=0;
  Var<floatA> depth;
//  Var<arr> points;
  Var<byteA> color;
  arr depth_fxypxy, color_fxypxy;

  RealSenseThread();
  ~RealSenseThread();
  void open();
  void close();
  void step();
};
