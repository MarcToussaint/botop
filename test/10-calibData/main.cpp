#include <Franka/controlEmulator.h>
#include <Franka/franka.h>
#include <Franka/help.h>

#include <Algo/SplineCtrlFeed.h>

#include <Kin/frame.h>

#include <KOMO/komo.h>
#include <Kin/F_qFeatures.h>
#include <Kin/viewer.h>

#include <BotOp/bot.h>

//===========================================================================

void driveToPoses() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));

  //-- load a file of poses
  arr X;
  X <<FILE("poses.dat");

  LOG(0) <<"LOADED POSES: #" <<X.d0 <<" (" <<X.d1 <<"-dimensional)";

  uint K=X.d0, Kmax=50;
  if(K>Kmax) K=Kmax;

  BotOp bot(C, !rai::checkParameter<bool>("sim"));
  bot.robot->writeData=0;

  rai::wait();

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

  driveToPoses();

  return 0;
}
