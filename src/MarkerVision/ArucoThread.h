#pragma once

#include <Core/thread.h>
#include <Core/array.h>
#include <Perception/aruco.h>
#include <Perception/MultiViewProblems.h>

namespace rai{

struct ArucoThread : Thread {
  Var<byteA>& input;
  Var<PointViewA> output;
  uint k_id;
  int input_revision=0;
  FindArucos finder;

  ArucoThread(uint k_id, Var<byteA>& _input);
  ~ArucoThread();

  void step();

private:
  byteA rgb;
};

} //rai
