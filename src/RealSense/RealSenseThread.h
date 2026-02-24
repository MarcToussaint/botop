#pragma once

#include <Core/array.h>
#include <Core/util.h>
#include <Core/thread.h>
#include <Control/CtrlMsgs.h>

namespace rs2 { class pipeline; }

struct RealSenseCfg{
  RAI_PARAM("RealSense/", int, startupSkipImages, 30)
  RAI_PARAM("RealSense/", int, resolution, 640)
  RAI_PARAM("RealSense/", bool, alignToDepth, true)
  RAI_PARAM("RealSense/", bool, autoExposure, true)
  RAI_PARAM("RealSense/", double, exposure, 500)
  RAI_PARAM("RealSense/", double, white, 4000)
  RAI_PARAM("RealSense/", bool, longCable, false)
};

struct RealSenseThread : Thread, rai::CameraAbstraction {
  struct sRealSenseThread *s=0;
  RealSenseCfg cfg;
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
