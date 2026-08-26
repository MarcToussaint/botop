#pragma once

#include <Core/thread.h>
#include <Core/array.h>
#include <Kin/kin.h>
#include <Perception/aruco.h>
#include <Perception/MultiViewProblems.h>

namespace rai{

struct ArucoSystemThread : Thread {
  rai::Array<Var<PointViewData>*> inputs;
  Var<arr> X;
  MultiViewSolver solver;

  ArucoSystemThread(uint J_numMarkers,
                    const rai::Array<Var<PointViewData>*>& ar_outputs,
                    arrA Fxycxy,
                    rai::Array<rai::Transformation> Pose);
  ~ArucoSystemThread();

  void step();

  void pull(rai::Configuration& C);

private:
  byteA rgb;
  FrameL frames;
};

} //rai
