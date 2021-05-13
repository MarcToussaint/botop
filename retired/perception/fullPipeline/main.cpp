#include <Kin/kin.h>

#include <KOMO/komo.h>
#include <KOMO/komo-ext.h>

#include <Perception/depth2PointCloud.h>
#include <Perception/perceptViewer.h>
#include <Perception/perceptSyncer.h>

#include <Kin/cameraview.h>
#include <Kin/kinViewer.h>
#include <Gui/viewer.h>

#include <BackgroundSubtraction/cv_depth_backgroundSubstraction.h>

#include <Control/TaskControlThread.h>
#include <Franka/controlEmulator.h>

#include <RealSense/RealSenseThread.h>

#include <LGPop/lgpop.h>

//===========================================================================

void perceptionPipeline(){
  LGPop OP(false);

  OP.runCamera(0);
  OP.runPerception(0);
  rai::wait();


//  //cheating! in real I could not copy the real world!
//  K.set()->addFile("../../model/stick.g", "table", rai::Transformation({.0,-.5,.05}, 0));

//  rai::wait();

}

//===========================================================================

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  perceptionPipeline();

  return 0;
}


