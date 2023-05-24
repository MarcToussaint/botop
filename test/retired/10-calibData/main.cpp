
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
  komo.setConfig(C, true);
  komo.setTiming(1., 32, 5., 2);
  komo.add_qControlObjective({}, 2, 1.);

//  komo.addObjective({.4,.6}, FS_qItself, {}, OT_sos, {1.}, qHome);
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e0}, target_q);
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e0}, {}, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e2});

//  komo.initWithWaypoints({target_q});
//  komo.initWithConstant(target_q);
  komo.optimize(.1, OptOptions().set_stopTolerance(1e-3));

  //  cout <<komo.getReport(true) <<endl;
  cout <<"  path -- time:" <<komo.timeTotal <<"\t sos:" <<komo.sos <<"\t ineq:" <<komo.ineq <<"\t eq:" <<komo.eq <<endl;

  arr path = komo.getPath_qOrg();

  if(komo.sos>50. || komo.ineq>.1 || komo.eq>.1){
    FILE("z.path") <<path <<endl;
    cout <<komo.getReport(true) <<endl;
    LOG(-2) <<"WARNING!";
    while(komo.view_play(true));
#if 0
    //repeat
    komo.initWithConstant(q0);
    komo.optimize(.1, OptOptions().set_stopTolerance(1e-3));
#endif

    return {};
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
//    C.view(false, STRING("conf " <<i));
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

  BotOp bot(C, rai::getParameter<bool>("real", false));
  bot.robotL->writeData=0;

  bot.home(C);

  for(uint k=kStart;k<Kend;k++){
    cout <<"========== POSE " <<k <<" ===========" <<endl;
    C.setJointState(bot.get_q());
    arr path = getStartGoalPath(C, X[k], bot.qHome);
    if(!path.N) continue;
    bot.moveAutoTimed(path);
//    bot.moveTo(X[k], 3.);
    while(bot.sync(C));
    if(bot.keypressed=='q') break;
    bot.robotL->writeData=2;
    rai::wait(.1);
    bot.robotL->writeData=0;
  }

}

//===========================================================================

void driveToPoses(){
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  //  C["R_finger1"]->setShape(rai::ST_capsule, {0.02, 0.05}); //make fingers huge
  //  C["R_finger2"]->setShape(rai::ST_capsule, {0.02, 0.05}); //make fingers huge
  //  C["R_panda_coll_hand"]->setShape(rai::ST_capsule, {0.14, 0.08}); //make thing 3cm larger

  //-- load configurations
  arr X;
  X <<FILE("poses.dat");

  //-- fix poses
  fixPoses(X, C);

  driveToPoses(C, X);
}

//===========================================================================

void rndPoses(){
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));

  arr qHome = C.getJointState();
  arr limits = C.getLimits();
  FrameL collisionPairs = C.getCollisionAllPairs();
  for(uint i=0;i<collisionPairs.d0;i++){
    cout <<"PAIR: " <<collisionPairs(i,0)->name <<' ' <<collisionPairs(i,1)->name <<endl;
  }

  BotOp bot(C, rai::getParameter<bool>("real", false));
  bot.home(C);
  arr bounds = ~C.getLimits();

  uint N=100;
  for(uint i=0;i<N;i++){
    arr x = bounds[0] + (bounds[1]-bounds[0]) % rand(qHome.N);

    C.setJointState(x);
    //C.view(false, STRING("conf " <<i));
    bool succ = checkCollisionsAndLimits(C, collisionPairs, limits, true);
    if(succ){
      x = C.getJointState();
      cout <<" === sending POSE " <<i <<" === " <<endl;
      //C.view(true, STRING("conf " <<i));

      C.setJointState(bot.get_q());
      arr path = getStartGoalPath(C, x, bot.qHome);
      if(!path.N){
        LOG(0) <<"no path found - discarding pose " <<i;
        continue;
      }
      bot.moveAutoTimed(path);
      while(bot.sync(C));
      if(bot.keypressed=='q') break;
    }
  }
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  rnd.clockSeed();

  //  driveToPoses();
  rndPoses();

  return 0;
}
