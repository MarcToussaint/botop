#pragma once

#include <Core/array.h>
#include <Core/thread.h>
#include <unordered_map>

namespace rs2 {
struct config;
struct pipeline;
struct align;
}

namespace rai {

extern std::unordered_map<std::string, std::string> cameraMapping;

struct RealSenseCamera {
  bool captureColor;
  bool captureDepth;
  rai::String serialNumber;
  std::shared_ptr<rs2::config> cfg;
  std::shared_ptr<rs2::pipeline> pipe;
  std::shared_ptr<rs2::align> align;
  float depth_scale;
  arr fxycxy, color_fxycxy, depth_fxycxy;

  RealSenseCamera(rai::String serialNumber, bool captureColor, bool captureDepth);
};

struct MultiRealSenseThread : Thread {
  StringA serialNumbers;
  rai::Array<RealSenseCamera*> cameras;

  rai::Array<Var<byteA>> color;
  rai::Array<Var<floatA>> depth;
  bool captureColor;
  bool captureDepth;

  MultiRealSenseThread(const StringA& serialNumbers,
                       bool captureColor, bool captureDepth);
  ~MultiRealSenseThread();

  uint getNumberOfCameras();

  void open();
  void close();
  void step();
};

}
