#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>
#include <KOMO/pathTools.h>
#include <Optim/MP_Solver.h>
#include <Control/flagHunter.h>
#include <Core/thread.h>

#include "tosca.h"


//===========================================================================

rai::Frame& setupWorld(rai::Configuration& C){
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));

  rai::Frame& target =
      C.addFrame("target")
      ->setShape(rai::ST_ssBox, {.06,.15,.09,.01})
      .setPosition(arr{-.3,-.2,.695});

  C.addFrame("plate", "table")
      ->setJoint(rai::JT_rigid)
      .setShape(rai::ST_ssBox, {.4,.4,.1,.01})
      .setRelativePosition(arr{-.4,.2,.0});

  return target;
}

//===========================================================================

void testPnp2() {
  rai::Configuration C;
  rai::Frame& target = setupWorld(C);
  arr center = C["l_gripper"]->getPosition();
  arr qHome = C.getJointState();

  //-- define constraints
  uint K=2;

  const char* gripperName="l_gripper";
  const char* palmName="l_palm";
  const char* boxName="target";
  const char* tableName="plate";
  const char* arm1Name="l_panda_coll7";
  const char* arm2Name="l_panda_coll6";

  rai::Array<ObjectiveL> phiflag(K);
  rai::Array<ObjectiveL> phirun(K);

  arr boxSize={.06,.15,.09};
  rai::ArgWord dir = rai::_xAxis;
  arr xLine, yzPlane;
  FeatureSymbol xyScalarProduct=FS_none, xzScalarProduct=FS_none;
  getGraspLinePlane(xLine, yzPlane, xyScalarProduct, xzScalarProduct, dir);

  // generic collisions
  //komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1});

  //fixed pre-grasp distance
  phiflag(0).add({}, FS_distance, C, {gripperName, boxName}, OT_eq, {1e1}, {-.1});

  //position: center in inner target plane; X-specific
  phirun (1).add({}, FS_positionRel, C, {gripperName, boxName}, OT_eq, xLine*1e2, arr{});
  phiflag(1).add({}, FS_positionRel, C, {gripperName, boxName}, OT_ineq, yzPlane*1e2, (boxSize/2.-.02));
  phiflag(1).add({}, FS_positionRel, C, {gripperName, boxName}, OT_ineq, yzPlane*(-1e2), -(boxSize/2.-.02));

  //orientation: grasp axis orthoginal to target plane; X-specific
  phirun(1).add({}, xyScalarProduct, C, {gripperName, boxName}, OT_eq, {1e2}, {});
  phirun(1).add({}, xzScalarProduct, C, {gripperName, boxName}, OT_eq, {1e2}, {});

  //no collision with palm
  phirun(0).add({}, FS_distance, C, {palmName, boxName}, OT_ineq, {1e1}, {});
//  phirun(1).add({}, FS_distance, C, {palmName, boxName}, OT_ineq, {1e1}, {-.001});


  //-- MPC stuff
  TOSCA tosca(C, phiflag, phirun, qHome);

  //-- start a robot thread
  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.home(C);
  double ctrlTime = 0.;
  bot.setControllerWriteData(2);

  //-- iterate
  Metronome tic(.1);
  for(uint t=0;t<2000;t++){
    tic.waitForTic();

    //-- switch target randomly at some times
    if(!(t%20)){
      tosca.timingMPC.phase=0;
      tosca.timingMPC.tau = 10.;

      switch(rnd(4)){
        case 0: target.setPosition(center + arr{+.3,.0,+.2}); break;
        case 1: target.setPosition(center + arr{-.3,.0,+.2}); break;
        case 2: target.setPosition(center + arr{+.3,.0,-.2}); break;
        case 3: target.setPosition(center + arr{-.3,.0,-.2}); break;
      }
      LOG(0) <<"new target";
    }

    //-- get current state (time,q,qDot)
    arr q,qDot;
    bot.getState(q, qDot, ctrlTime);
    tosca.cycle(C, q, qDot, ctrlTime);
    tosca.report(C, phiflag, phirun);

    //send leap target
    if(tosca.pathMPC.feasible){
      if(!tosca.timingMPC.done()){
        arr times = tosca.timingMPC.getTimes();
        times -= bot.get_t() - ctrlTime;
        bot.move(tosca.timingMPC.getFlags(), tosca.timingMPC.getVels(), times, true);
      }
    }

    //update C
    bot.step(C, .0);
    if(bot.keypressed==13){ t=9; continue; }
    if(bot.keypressed=='q' || bot.keypressed==27) return;
  }
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

//  rnd.clockSeed();
  rnd.seed(1);

  testPnp2();

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::getParameters()() <<'\n';

  return 0;
}
