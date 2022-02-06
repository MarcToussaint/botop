#pragma once

#include <Core/array.h>
#include <Core/thread.h>

namespace rs2 {
struct config;
struct pipeline;
struct align;
}

namespace rai {
namespace realsense {

struct RealSenseCamera {
  std::string serialNumber;
  bool captureColor;
  bool captureDepth;
  std::shared_ptr<rs2::config> cfg;
  std::shared_ptr<rs2::pipeline> pipe;
  std::shared_ptr<rs2::align> align;
  float depth_scale;
  arr fxypxy, color_fxypxy, depth_fxypxy;

  RealSenseCamera(std::string serialNumber, bool captureColor, bool captureDepth);
};

struct MultiRealSenseThread : Thread {
  std::vector<std::string> serialNumbers;
  Var<std::vector<byteA>> color;
  Var<std::vector<floatA>> depth;
  bool captureColor;
  bool captureDepth;

  std::vector<RealSenseCamera*> cameras;

  MultiRealSenseThread(const std::vector<std::string> serialNumbers,
                       const Var<std::vector<byteA>>& color,
                       const Var<std::vector<floatA>>& depth,
                       bool captureColor, bool captureDepth);
  ~MultiRealSenseThread();

  uint getNumberOfCameras();

  void open();
  void close();
  void step();
};

}
}
