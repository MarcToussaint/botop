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

arr getStartGoalPath(rai::Configuration& C, const arr& target_q, const arr& qHome) {

  arr q0 = C.getJointState();

  KOMO komo;
  komo.setModel(C, true);
  komo.setTiming(1., 32, 3., 2);
  komo.add_qControlObjective({}, 2, 1.);

  komo.addObjective({.4,.6}, FS_qItself, {}, OT_sos, {1.}, qHome);
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e0}, target_q);
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e0}, {}, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e2});

//  komo.initWithWaypoints({target_q});
//  komo.initWithConstant(target_q);
  komo.optimize(0.);

  //  cout <<komo.getReport(true) <<endl;
  cout <<"  path -- time:" <<komo.timeTotal <<"\t sos:" <<komo.sos <<"\t ineq:" <<komo.ineq <<"\t eq:" <<komo.eq <<endl;

  arr path = komo.getPath_qOrg();
//  FILE("z.path") <<path <<endl;
//  while(komo.view_play(true));

  if(komo.sos>50. || komo.ineq>.2 || komo.eq>.2){
    FILE("z.path") <<path <<endl;
    cout <<komo.getReport(true) <<endl;
    LOG(-2) <<"WARNING!";
    while(komo.view_play(true));
  }

  return path;
}

//===========================================================================

void fixPoses(arr& X, rai::Configuration& C){

  //-- load a file of poses
  LOG(0) <<"LOADED POSES: #" <<X.d0 <<" (" <<X.d1 <<"-dimensional)";

  arr qHome = C.getJointState();
  arr limits = C.getLimits();
  FrameL collisionPairs = C.getCollisionAllPairs();
  for(uint i=0;i<collisionPairs.d0;i++){
    cout <<"PAIR: " <<collisionPairs(i,0)->name <<' ' <<collisionPairs(i,1)->name <<endl;
  }

  for(uint i=0;i<X.d0;i++){
    C.setJointState(X[i]);
//    C.watch(false, STRING("conf " <<i));
    bool succ = checkCollisionsAndLimits(C, collisionPairs, limits, true);
    if(succ) X[i] = C.getJointState();
    else X[i] = X[i-1];
  }

  C.setJointState(qHome);
}

//===========================================================================

void driveToPoses(rai::Configuration& C, const arr& X, const uint kStart=0) {
  uint Kend=X.d0, Kmax=5000;
  if(Kend>Kmax) Kend=Kmax;

  BotOp bot(C, !rai::checkParameter<bool>("sim"));
  bot.robot->writeData=0;
  if(absMax(bot.get_q()-bot.qHome)>.01){
    bot.moveLeap(bot.qHome);
  }


  for(uint k=kStart;k<Kend;k++){
    cout <<"========== POSE " <<k <<" ===========" <<endl;
    C.setJointState(bot.get_q());
    arr path = getStartGoalPath(C, X[k], bot.qHome);
    bot.move(path, {3.});
//    bot.moveLeap(X[k], 3.);
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

  //-- load configurations
  arr X;
  X <<FILE("poses.dat");

  //-- fix poses
  fixPoses(X, C);

  driveToPoses(C, X, 30);

  return 0;
}
