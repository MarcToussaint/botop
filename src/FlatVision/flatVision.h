#pragma once

//#include <Perception/percept.h>
#include <Core/thread.h>

#include "helpers.h"
#include "explainBackground.h"
#include "explainRobot.h"
#include "explainNovels.h"
#include "objectManager.h"

//-- thread wrapper
struct FlatVisionThread : Thread {
  //output
//  Var<PerceptL> percepts;
  Var<rai::KinematicWorld> config;
  //input
  Var<byteA> cam_color;
  Var<floatA> cam_depth;
  Var<byteA> model_segments;
  Var<floatA> model_depth;
  Var<arr> cam_pose;
  Var<arr> cam_Fxypxy;
  //-- calibration variables
  Var<arr> armPoseCalib; //(2x6 matrix: (dx,dy,dz, rx,ry,rz) (with trans and infinitesimal rot; for both arms)
  //parameters
  int verbose=1;

  //methods
  ExplainBackground exBackground;
  ExplainRobotPart exRobot;
  ExplainNovelPercepts exNovel;
  ObjectManager objectManager;

  FlatVisionThread(Var<rai::KinematicWorld>& _config,
                      Var<byteA>& _color,
                      Var<floatA>& _depth,
                      Var<byteA>& _model_segments,
                      Var<floatA> _model_depth,
                      Var<arr>& _cameraPose,
                      Var<arr>& _cameraFxypxy,
                      Var<arr>& _armPoseCalib,
                      int _verbose=1);
  ~FlatVisionThread(){
    threadClose();
  }
  void step();
};
