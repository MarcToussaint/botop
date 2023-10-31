#include "tools.h"

#include <Optim/NLP_Solver.h>
#include <Gui/opengl.h>
#include <MarkerVision/cvTools.h>
#include <Geo/depth2PointCloud.h>

Move_IK::Move_IK(BotOp& _bot, rai::Configuration& _C, int _askForOK) : bot(_bot), C(_C), askForOK(_askForOK){
  komo.setConfig(C, true);
  komo.setTiming(1., 1, 1., 0);
  komo.addControlObjective({}, 0, 1e-1);
}

bool Move_IK::go(){
  auto ret = NLP_Solver().setProblem(komo.nlp()).solve();
  cout <<*ret <<endl;
  if(!ret->feasible){
    return false;
  }else{
    qT = komo.getPath_qOrg()[-1];
    C.setJointState(qT);
    if(askForOK && C.view(true, "Move_IK\ngo?")=='q') return false;

    bot.moveTo(qT, 2.);
    for(;bot.getTimeToEnd()>0.;) bot.sync(C);
  }
  return true;
}

bool sense_HsvBlob(BotOp& bot, rai::Configuration& C, const char* camName, const char* blobName, const arr& hsvFilter, const arr& Pinv, int verbose){
  OpenGL disp;
  byteA img;
  floatA depth;
  bot.getImageAndDepth(img, depth, camName);
  arr u = getHsvBlobImageCoords(img, depth, hsvFilter, verbose-1);
  if(verbose>1) disp.watchImage(img, false);
  if(verbose>0) LOG(0) <<"dot in image coords: " <<u;
  if(!u.N) return false;

  arr x;
  if(Pinv.N){
    makeHomogeneousImageCoordinate(u);
    x = Pinv*u;
  }else{
    x = u;
    depthData2point(x, bot.getCameraFxycxy(camName));
  }
  if(verbose>0) LOG(0)  <<"dot in cam coords: " <<x;
  C[camName]->get_X().applyOnPoint(x);
  if(verbose>0) LOG(0)  <<"dot in world coords: " <<x;

  C[blobName]->setPosition(x);
  if(verbose>0 && C.view(true, "sense_HsvBlob\ngo?")=='q') return false;

  return true;
}
