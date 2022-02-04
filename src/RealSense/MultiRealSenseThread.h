#pragma once

#include <Core/array.h>
#include <Core/thread.h>

namespace rai {
namespace realsense {

struct RealSenseCamera;

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
