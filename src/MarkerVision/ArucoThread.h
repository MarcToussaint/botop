#pragma once

#include <Core/thread.h>
#include <Core/array.h>
#include <Perception/aruco.h>
#include <Perception/MultiViewProblems.h>

namespace rai{

struct ArucoOutput{ uint cam_id; intA ids; arr pts; };

struct ArucoThread : Thread {
  Var<byteA>& input;
  Var<ArucoOutput> output;
  uint cam_id;
  int input_revision=0;
  FindArucos finder;

  ArucoThread(uint cam_id, Var<byteA>& _input, double beatIntervalSec=0.025);
  ~ArucoThread();

  void step();

private:
  byteA rgb;
};

} //rai
