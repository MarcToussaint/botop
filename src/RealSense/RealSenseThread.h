#pragma once

#include <Core/array.h>
#include <Core/thread.h>
#include <Control/CtrlMsgs.h>

namespace rs2 { class pipeline; }

struct RealSenseThread : Thread, rai::CameraAbstraction {
  struct sRealSenseThread *s=0;
  Var<byteA> image;
  Var<floatA> depth;
  arr fxycxy, color_fxycxy, depth_fxycxy;

  RealSenseThread(const char *_name);
  ~RealSenseThread();

  virtual void getImageAndDepth(byteA& _image, floatA& _depth);
  arr getFxycxy(){ return fxycxy; }

protected:
  void open();
  void close();
  void step();
};
