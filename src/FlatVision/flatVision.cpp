#include "flatVision.h"

FlatVisionThread::FlatVisionThread(Var<rai::KinematicWorld>& _config,
                                   Var<rai::Array<ptr<Object>>>& _objects,
                                   Var<byteA>& _color, Var<floatA>& _depth,
                                   Var<byteA>& _model_segments, Var<floatA> _model_depth,
                                   Var<uintA>& _cameraCrop, Var<arr>& _cameraPInv, Var<arr>& _armPoseCalib,
                                   int _verbose)
  : Thread("FlatVision", -1.),
    config(this, _config),
    objects(this, _objects),
    cam_color(this, _color),
    cam_depth(this, _depth, true),
    model_segments(this, _model_segments),
    model_depth(this, _model_depth),
    cam_crop(this, _cameraCrop),
    cam_PInv(this, _cameraPInv),
    armPoseCalib(this, _armPoseCalib),
    verbose(_verbose),
    objectManager(_objects){
  exBackground.verbose = verbose;
  exRobot.verbose = verbose;
  exNovel.verbose = verbose;
  threadOpen();
//  threadStep();
//  threadLoop();
}

void FlatVisionThread::step(){
  byteA labels;
  byteA _cam_color = cam_color.get();
  floatA _cam_depth = cam_depth.get();
  byteA _model_segments = model_segments.get();
  floatA _model_depth = model_depth.get();
  arr _cam_PInv = cam_PInv.get();

  //not ready yet?
  if(_cam_depth.nd!=2 || _model_segments.nd!=2){
    return;
  }


  //-- crop
  //uint cL=95, cR=80, cT=50, cB=30;

  //uint cL=95, cR=80, cT=80, cB=10;
  uintA _camCrop = cam_crop.get();
  uint cL = _camCrop(0), cR = _camCrop(1), cT = _camCrop(2), cB = _camCrop(3);

  _cam_color = _cam_color.sub(cT,-cB,cL,-cR,0,-1);
  _cam_depth = _cam_depth.sub(cT,-cB,cL,-cR);
  _model_segments = _model_segments.sub(cT,-cB,cL,-cR);
  _model_depth = _model_depth.sub(cT,-cB,cL,-cR);

  // TODO this assumes that the calibration was done with the already cropped image!
  // This of course can be fixed easily in the project method, which should get the cropping parameters

//  _cam_fxypxy(2) -= cL;
//  _cam_fxypxy(3) -= cT;


  //-- background
  exBackground.computeBackground = updateBackground;
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

  //-- novel percepts
  exNovel.compute(labels, _cam_color, _cam_depth);

  //just render, for display only
  objectManager.renderFlatObject(labels.d0, labels.d1);

  //decide pixel-wise which pixels of novel percepts to merge into existing objects
  objectManager.assignPerceptsToObjects(exNovel.flats, labels);

  //add remaining novel objects as objects
  objectManager.injectNovelObjects(exNovel.flats, labels,
                                   _cam_color, _cam_depth);

  //adapt objects based on novel pixels
  objectManager.adaptFlatObjects(labels, _cam_color, _cam_depth, _camCrop, _cam_PInv, exBackground.background);

  if(syncToConfig){
    objectManager.removeUnhealthyObject(config.set());
    objectManager.syncWithConfig(config.set());
  }

  objectManager.displayLabels(labels, _cam_color);

  if(verbose>1){
    objectManager.printObjectInfos();
  }

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

