#pragma once

#include <Core/array.h>
#include <Core/thread.h>
#include <Perception/percept.h>
#include <Perception/depth2PointCloud.h>

struct CV_BackgroundSubstraction_Percept{
  double x=0.,y=0.;
  double radius=0.;
  arr rect;
  arr polygon;
  floatA depthRect;
  floatA backgroundRect;
};

//-- pure algo
struct CV_BackgroundSubstraction{
  //parameters
  float threshold=.01;
  float depthcut = 1.05;
  int verbose=1;
  //internal
  byteA noSignal;
  byteA countDeeper;
  floatA valueDeeper;
  byteA countStable;
  //output
  floatA background;
  byteA mask;
  rai::Array<CV_BackgroundSubstraction_Percept> CVpercepts;

  void compute(const byteA& _color, const floatA& _depth, const byteA& _inputMask);
};

//-- thread wrapper
struct CV_BackgroundSubstraction_Thread : Thread, CV_BackgroundSubstraction {
  //output
  Var<PerceptL> percepts;
  //input
  Var<byteA> color;
  Var<floatA> depth;
  Var<byteA> mask;
  Var<arr> cameraPose;
  Var<arr> cameraFxypxy;
  CV_BackgroundSubstraction_Thread(Var<PerceptL>& _percepts,
                                   Var<byteA>& _color,
                                   Var<floatA>& _depth,
                                   Var<byteA>& _mask,
                                   Var<arr>& _cameraPose,
                                   Var<arr>& _cameraFxypxy, int _verbose=1)
    : Thread("BackgroundSubstraction", -1.),
      percepts(this, _percepts),
      color(this, _color),
      depth(this, _depth, true),
      mask(this, _mask),
      cameraPose(this, _cameraPose),
      cameraFxypxy(this, _cameraFxypxy){
    CV_BackgroundSubstraction::verbose = _verbose;
    threadOpen();
  }
  ~CV_BackgroundSubstraction_Thread(){
    threadClose();
  }
  void step();
};
