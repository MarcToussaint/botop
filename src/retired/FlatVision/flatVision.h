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
  Var<rai::Array<ptr<Object>>> objects;
  //input
  Var<byteA> cam_color;
  Var<floatA> cam_depth;
  Var<byteA> model_segments;
  Var<floatA> model_depth;
  Var<uintA> cam_crop;
  Var<arr> cam_PInv;
  //-- calibration variables
  Var<arr> armPoseCalib; //(2x6 matrix: (dx,dy,dz, rx,ry,rz) (with trans and infinitesimal rot; for both arms)
  //parameters
  int verbose=1;
  bool syncToConfig=true;

  bool updateBackground = true;

  //methods
  ExplainBackground exBackground;
  ExplainRobotPart exRobot;
  ExplainNovelPercepts exNovel;
  ObjectManager objectManager;

  FlatVisionThread(Var<rai::KinematicWorld>& _config,
                   Var<rai::Array<ptr<Object>>>& _objects,
                   Var<byteA>& _color,
                   Var<floatA>& _depth,
                   Var<byteA>& _model_segments,
                   Var<floatA> _model_depth,
                   Var<uintA>& _cameraCrop,
                   Var<arr>& _cameraPInv,
                   Var<arr>& _armPoseCalib,
                   int _verbose=1);
  ~FlatVisionThread(){
    threadClose();
  }
  void step();
};
