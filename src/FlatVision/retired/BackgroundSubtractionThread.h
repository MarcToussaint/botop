#pragma once

#include <Core/array.h>
#include <Core/thread.h>

namespace cv {
  struct Mat;
  struct BackgroundSubtractorMOG2;
}

struct BackgroundSubtractionThread : Thread {
  Var<byteA> color;
  Var<cv::Mat> mask;

  double rate;

  bool updateBackgroundModel = true;

  bool morphologicalCleaning = true;
  uint mSize = 3;
  bool contourFilling = true;

  cv::BackgroundSubtractorMOG2* bgs;

  BackgroundSubtractionThread(Var<byteA>& color, double rate = 0.01);
  virtual ~BackgroundSubtractionThread();

  void learnBackgroundModel(uint nFrames = 100);

  void open();
  void close();
  void step();
};

