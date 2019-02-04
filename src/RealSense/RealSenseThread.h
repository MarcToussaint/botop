#pragma once

#include <Core/array.h>
#include <Core/thread.h>

namespace rs2 { class pipeline; }

struct RealSenseThread : Thread {
  struct sRealSenseThread *s=0;
  Var<byteA> color;
  Var<floatA> depth;
  arr depth_fxypxy, color_fxypxy;

  RealSenseThread(const Var<byteA>& _color, const Var<floatA>& _depth);
  ~RealSenseThread();
  void open();
  void close();
  void step();

  arr getFxypxy(){ return depth_fxypxy; }
};
