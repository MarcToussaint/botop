#pragma once

#include <Core/array.h>
#include <Core/thread.h>
#include <Control/CtrlMsgs.h>

namespace rs2 { class pipeline; }

struct RealSenseThread : Thread, rai::CameraAbstraction {
  struct sRealSenseThread *s=0;
  Var<byteA> image;
  Var<floatA> depth;
  arr fxypxy, color_fxypxy, depth_fxypxy;

  RealSenseThread(const char *_name);
  ~RealSenseThread();

  virtual void getImageAndDepth(byteA& _image, floatA& _depth){
    image.waitForRevisionGreaterThan(10);
    depth.waitForRevisionGreaterThan(10);
    _image = image.get();
    _depth = depth.get();
  }
  arr getFxypxy(){ return fxypxy; }

protected:
  void open();
  void close();
  void step();

};
