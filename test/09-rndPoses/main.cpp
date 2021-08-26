
#include <KOMO/komo.h>
#include <KOMO/pathTools.h>
#include <Kin/F_qFeatures.h>
#include <Kin/viewer.h>

#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>

//===========================================================================

//arr getStartGoalPath(rai::Configuration& C, const arr& qTarget, const arr& qHome, const rai::Array<Avoid>& avoids={}, const char* endeff=0, bool endeffApproach=false, bool endeffRetract=false);
arr getStartGoalPath2(rai::Configuration& C, const arr& target_q, const arr& qHome, const rai::Array<Avoid>& avoids={}, uint trials=3) {

  arr q0 = C.getJointState();

  KOMO komo;
  komo.setModel(C, true);
  komo.setTiming(1., 32, 5., 2);
  komo.add_qControlObjective({}, 2, 1.);

  //constrain target - either endeff target or qTarget
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e0}, target_q);

  //final still
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e0}, {}, 1);

  //homing
  if(qHome.N) komo.addObjective({.4,.6}, FS_qItself, {}, OT_sos, {1e-1}, qHome);

  // collision avoidances
#if 0 //explicit enumeration -- but that's inefficient for large scenes; the iterative approach using accumulated is more effective
  if(collisionPairs.N){
    komo.addObjective({}, FS_distance, framesToNames(collisionPairs), OT_ineq, {1e2}, {-.001});
  }
#else
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1});
#endif

//  for(const Avoid& a:avoids){
//    komo.addObjective(a.times, FS_distance, a.frames, OT_ineq, {1e1}, {-a.dist});
//  }

  //-- run several times with random initialization
  bool feasible=false;
  for(uint trial=0;trial<trials;trial++){
    //initialize with constant q0 or target_q
    komo.reset();
    if(trial%2) komo.initWithConstant(target_q);
    else komo.initWithConstant(q0);

    //optimize
    komo.optimize(.02, OptOptions().set_stopTolerance(1e-3));

    //is feasible?
    feasible=komo.sos<50. && komo.ineq<.1 && komo.eq<.1;

    //if not feasible -> add explicit collision pairs (from proxies presently in komo.pathConfig)
    if(!feasible){
      cout <<komo.getReport(false);
      //komo.pathConfig.reportProxies();
      StringA collisionPairs = komo.getCollisionPairs(.01);
      if(collisionPairs.N){
        komo.addObjective({}, FS_distance, collisionPairs, OT_ineq, {1e2}, {-.001});
      }
    }

    cout <<"  path trial " <<trial <<" -- time:" <<komo.timeTotal <<"\t sos:" <<komo.sos <<"\t ineq:" <<komo.ineq <<"\t eq:" <<komo.eq <<endl;
    if(feasible) break;
  }

  if(!feasible) return {};

  arr path = komo.getPath_qOrg();

  return path;
}

//===========================================================================

void rndPoses(){
  rai::Configuration C;
  rai::String useArm = rai::getParameter<rai::String>("bot/useArm", "both");
  if(useArm=="both"){
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));
    C.addFrame("l_gripper_target") ->setShape(rai::ST_marker, {.2});
    C.addFrame("r_gripper_target") ->setShape(rai::ST_marker, {.2});
  }else{
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
    C.addFrame("l_gripper_target") ->setShape(rai::ST_marker, {.2});
  }

  arr qHome = C.getJointState();
  arr limits = C.getLimits();
  FrameL collisionPairs = C.getCollisionAllPairs();
  for(uint i=0;i<collisionPairs.d0;i++){
    cout <<"PAIR: " <<collisionPairs(i,0)->name <<' ' <<collisionPairs(i,1)->name <<endl;
  }

  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.home(C);
  arr bounds = ~C.getLimits();

  uint N=100;
  for(uint i=0;i<N;i++){
    cout <<" ====== POSE " <<i <<" ====== " <<endl;
    arr x = bounds[0] + (bounds[1]-bounds[0]) % rand(qHome.N);

    C.setJointState(x);
    //C.watch(false, STRING("conf " <<i));
    bool succ = checkCollisionsAndLimits(C, collisionPairs, limits, true);
    if(succ){
      cout <<" === pose made feasible === " <<endl;
      x = C.getJointState();
      //C.watch(true, STRING("conf " <<i));

      C.setJointState(bot.get_q());
      arr path = getStartGoalPath(C, x, bot.qHome); //, {}, {"l_gripper", "r_gripper"}, true, true);
      if(!path.N){
        cout <<" === path infeasible === " <<endl;
        continue;
      }
      cout <<" === path feasible -> executing === " <<endl;
      bot.moveAutoTimed(path, .01);
      while(bot.step(C));
      if(bot.keypressed=='q') break;
    }else{
      cout <<" === pose infeasible === " <<endl;
    }
  }
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

//  rnd.clockSeed();

  //  driveToPoses();
  rndPoses();

  return 0;
}
