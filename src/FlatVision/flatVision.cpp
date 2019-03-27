#include "flatVision.h"

FlatVisionThread::FlatVisionThread(Var<rai::KinematicWorld>& _config, Var<byteA>& _color, Var<floatA>& _depth, Var<byteA>& _model_segments, Var<floatA> _model_depth, Var<arr>& _cameraPose, Var<arr>& _cameraFxypxy, Var<arr>& _armPoseCalib, int _verbose)
  : Thread("FlatVision", .1),
    config(this, _config),
    cam_color(this, _color),
    cam_depth(this, _depth, true),
    model_segments(this, _model_segments),
    model_depth(this, _model_depth),
    cam_pose(this, _cameraPose),
    cam_Fxypxy(this, _cameraFxypxy),
    armPoseCalib(this, _armPoseCalib),
    verbose(_verbose){
  exBackground.verbose = verbose;
  exRobot.verbose = verbose;
  exNovel.verbose = verbose;
//  threadOpen();
  threadStep();
}

void FlatVisionThread::step(){
  byteA labels;
  byteA _cam_color = cam_color.get();
  floatA _cam_depth = cam_depth.get();
  byteA _model_segments = model_segments.get();
  floatA _model_depth = model_depth.get();
  arr _cam_fxypxy = cam_Fxypxy.get();
  arr _cam_pose = cam_pose.get();

  //not ready yet?
  if(_cam_depth.nd!=2 || _model_segments.nd!=2){
    return;
  }

  //-- crop
  uint cL=100,cR=140,cT=100,cB=100;
  _cam_color = _cam_color.sub(cT,-cB,cL,-cR,0,-1);
  _cam_depth = _cam_depth.sub(cT,-cB,cL,-cR);
  _model_segments = _model_segments.sub(cT,-cB,cL,-cR);
  _model_depth = _model_depth.sub(cT,-cB,cL,-cR);
  _cam_fxypxy(2) -= cL;
  _cam_fxypxy(3) -= cT;

  //-- background
  exBackground.compute(labels, _cam_color, _cam_depth);

  //-- robot
#if 0
  exRobot.label=PL_robot;
  exRobot.compute(labels, _cam_color, _cam_depth, _model_segments, _model_depth);
  {
    //  cout <<"calib_L=" <<exRobot.calib <<endl;
    arr fxypxy = cam_Fxypxy.get();
    auto armCalib = armPoseCalib.set();
    armCalib()[0] = exRobot.calib;
    armCalib()(0,0) /= fxypxy(0);
    armCalib()(0,1) /= fxypxy(1);
  }

  objectManager.displayLabelPCL(exRobot.label,
                                labels, _cam_depth,
                                _cam_pose, _cam_fxypxy,
                                config.set());

  exRobot.label=PixelLabel(PL_robot|1);
  exRobot.compute(labels, _cam_color, _cam_depth, _model_segments, _model_depth);
  {
    //  cout <<"calib_R=" <<exRobot.calib <<endl;
    arr fxypxy = cam_Fxypxy.get();
    auto armCalib = armPoseCalib.set();
    armCalib()[1] = exRobot.calib;
    armCalib()(1,0) /= fxypxy(0);
    armCalib()(1,1) /= fxypxy(1);
  }

  objectManager.displayLabelPCL(exRobot.label,
                                labels, _cam_depth,
                                _cam_pose, _cam_fxypxy,
                                config.set());
#endif

  //-- 3D objects
#if 0
  exObject.label=PL_objects;
  exObject.verbose=1;
  exObject.compute(labels, _cam_color, _cam_depth, _model_segments, _model_depth);
  {
    objectManager.updateObjectPose(exObject.label, exObject.calib, _cam_fxypxy);
  }

  objectManager.explainObjectPixels(labels, _cam_color, _cam_depth, _model_segments, _model_depth);
#endif

  //-- flat objects
  objectManager.explainFlatObjectPixels(labels, _cam_color, _cam_depth, _model_segments, _model_depth);
  objectManager.adaptFlatObjects(labels, _cam_color, _cam_depth, _model_segments, _model_depth);

  //-- novel percepts
  exNovel.compute(labels, _cam_color, _cam_depth);
  if(exNovel.flats.N>0){
    objectManager.displayLabelPCL(PL_novelPercepts,
                                  labels, _cam_depth,
                                  _cam_pose, _cam_fxypxy,
                                  config.set());
  }

  //-- merge with 2D objects
#if 0
  objectManager.flatRenderObjects(_cam_depth.d0, _cam_depth.d1);

  objectManager.processNovelPercepts(exNovel.flats,
                                     labels,
                                     _cam_color, _cam_depth,
                                     _model_segments, _model_depth,
                                     _cam_pose, _cam_fxypxy);
#endif

  //-- map novels to objects
#if 1
  objectManager.injectNovelObjects(exNovel.flats,
                                   labels,
                                   _cam_color, _cam_depth,
                                   _cam_pose, _cam_fxypxy);

//  objectManager.syncWithConfig(config.set()());
#endif

  objectManager.displayLabels(labels, _cam_color);

  objectManager.printObjectInfos();

//  if(verbose>0){
//    LOG(0) <<"novels: #=" <<exNovel.flats.N;
//    for(FlatPercept& p : exNovel.flats){
//      cout <<" * FlatPercept: label=x" <<std::hex <<p.label
//          <<"\n    x=" <<p.x <<" y=" <<p.y <<" rad=" <<p.radius <<" rect=" <<p.radius <<endl;
//    }
//  }

//  //make objects from novel flat percepts
//  CHECK(_cam_fxypxy.N, "need camera calibration parameters");
//  if(true){
//    auto P = percepts.set();
//    P->clear();
//    for(FlatPercept& flat : exNovel.flats){
//      P->append( novelPerceptToObject(flat, labels, _cam_depth, _cam_pose, _cam_fxypxy) );
//    }
//  }
}

