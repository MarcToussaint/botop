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
namespace realsense {

extern std::unordered_map<std::string, std::string> cameraMapping;

struct RealSenseCamera {
  std::string cameraName;
  bool captureColor;
  bool captureDepth;
  std::string serialNumber;
  std::shared_ptr<rs2::config> cfg;
  std::shared_ptr<rs2::pipeline> pipe;
  std::shared_ptr<rs2::align> align;
  float depth_scale;
  arr fxycxy, color_fxycxy, depth_fxycxy;

  RealSenseCamera(std::string cameraName, bool captureColor, bool captureDepth);
};

struct MultiRealSenseThread : Thread {
  std::vector<std::string> cameraNames;
  Var<std::vector<byteA>> color;
  Var<std::vector<floatA>> depth;
  bool captureColor;
  bool captureDepth;

  std::vector<RealSenseCamera*> cameras;

  MultiRealSenseThread(const std::vector<std::string> cameraNames,
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
