#include <Franka/controlEmulator.h>
#include <Franka/franka.h>
#include <Franka/help.h>

#include <Algo/SplineCtrlFeed.h>

#include <Kin/frame.h>

#include <KOMO/komo.h>
#include <KOMO/pathTools.h>
#include <Kin/F_qFeatures.h>
#include <Kin/viewer.h>

#include <BotOp/bot.h>

//===========================================================================

arr loadAndFixPoses(rai::Configuration& C){

  //-- load a file of poses
  arr X;
  X <<FILE("poses.dat");

  LOG(0) <<"LOADED POSES: #" <<X.d0 <<" (" <<X.d1 <<"-dimensional)";

  arr limits = C.getLimits();
  FrameL collisionPairs = C.getCollisionAllPairs();

  for(uint i=0;i<X.d0;i++){
    C.setJointState(X[i]);
//    C.watch(false, STRING("conf " <<i));
    checkCollisionsAndLimits(C, collisionPairs, limits, true);
  }

  return X;
}

//===========================================================================

void driveToPoses(rai::Configuration& C, const arr& X) {
  uint K=X.d0, Kmax=5000;
  if(K>Kmax) K=Kmax;

  BotOp bot(C, !rai::checkParameter<bool>("sim"));
  bot.robot->writeData=0;

  for(uint k=0;k<K;k++){
    bot.moveLeap(X[k], 2.);
    while(bot.step(C));
    if(bot.keypressed=='q') break;
    bot.robot->writeData=2;
    rai::wait(.1);
    bot.robot->writeData=0;
  }

}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));

  arr X = loadAndFixPoses(C);

  driveToPoses(C, X);

  return 0;
}
