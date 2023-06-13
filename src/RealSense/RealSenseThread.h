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
    uint n=100;
    if(image.getRevision()<n){
      LOG(0) <<"waiting to get " <<n <<" images from RealSense for autoexposure settling";
      image.waitForRevisionGreaterThan(n); //need many starting images for autoexposure to get settled!!
      depth.waitForRevisionGreaterThan(n);
    }
    _image = image.get();
    _depth = depth.get();
  }
  arr getFxypxy(){ return fxypxy; }

protected:
  void open();
  void close();
  void step();

};
