#pragma once

#include <Core/array.h>
#include <Core/thread.h>

namespace rs2 { class pipeline; }

struct RealSenseThread : Thread {
  std::shared_ptr<rs2::pipeline> pipe;
  Var<arr> depth;
  Var<byteA> color;

  RealSenseThread();
  ~RealSenseThread();
  void open();
  void close();
  void step();
};
