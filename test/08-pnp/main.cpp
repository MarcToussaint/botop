#include <Franka/controlEmulator.h>
#include <Franka/franka.h>
#include <Franka/gripper.h>
#include <Franka/help.h>

#include <Algo/SplineCtrlFeed.h>

#include <Kin/frame.h>

#include <KOMO/komo.h>
#include <Kin/F_qFeatures.h>
#include <Kin/F_pose.h>
#include <Kin/viewer.h>
#include <Control/LeapMPC.h>

#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>

/* TODO:
 * startToGoal -> solver that returns PathQuery
 * pnpKeyframes -> same, with identical collision constraints
 */

//===========================================================================

void testPnp() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));

  //add a target object
  C.addFrame("target", "table")
      ->setJoint(rai::JT_rigid)
      .setShape(rai::ST_ssBox, {.06,.15,.09,.01})
      .setRelativePosition(arr{-.4,-.4,.075});
  arr qHome = C.getJointState();

  //-- start a robot thread
  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.home(C);

  const char* gripperName="r_gripper";
  const char* palmName="r_palm";
  const char* boxName="target";
  const char* tableName="table";

  //-- compute a path
  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(3, 10, 3., 2);
  komo.add_qControlObjective({}, 2, 1e-1);
//  komo.add_qControlObjective({}, 1, 1e-1);

  arr boxSize={.06,.15,.09};

  //-- pick
  komo.addModeSwitch({1.,2.}, rai::SY_stable, {gripperName, boxName}, true);
  addBoxPickObjectives(komo, 1., rai::_xAxis, boxName, boxSize, gripperName, palmName, tableName);

  //-- carry above table
  if(komo.k_order>1) komo.addObjective({1.2,1.8}, FS_distance, {boxName, tableName}, OT_ineq, arr{1e1}, {-.05});

  //-- place
  komo.addModeSwitch({2.,-1.}, rai::SY_stable, {"table", boxName}, false);
  addBoxPlaceObjectives(komo, 2., rai::_zAxis, boxName, boxSize, gripperName, palmName);

  //-- home
  komo.addObjective({3.}, FS_qItself, {}, OT_eq, {1e1}, qHome);
  if(komo.k_order>1) komo.addObjective({3.}, FS_qItself, {}, OT_eq, {1e1}, {}, 1);


  komo.optimize();
  komo.getReport(true);
  komo.view(true);
  while(komo.view_play(true, .5));


  //get KOMO parts
  arr fullPath = komo.getPath_qOrg();
  arrA path(komo.T/komo.stepsPerPhase);
  for(uint k=0;k<path.N;k++){
    path(k) = fullPath({k*komo.stepsPerPhase,(k+1)*komo.stepsPerPhase-1});
  }

  for(uint k=0;k<path.N;k++){
//    rai::wait();
    if(k==0){ bot.gripperL->open(); rai::wait(.3); }
    if(k==1){ bot.gripperL->close(); rai::wait(.5); }
    if(k==2){ bot.gripperL->open(); rai::wait(.3); }

    //send komo as spline:
    bot.move(path(k), {4.});

    while(bot.step(C));
    if(bot.keypressed=='q' || bot.keypressed==27) return;
  }

}

//===========================================================================

arr getPnpKeyframes(const rai::Configuration& C,
                    rai::ArgWord pickDirection, rai::ArgWord placeDirection,
                    const char* boxName, const char* gripperName, const char* palmName, const char* tableName,
                    const arr& qHome) {
  arr q0 = C.getJointState();

  KOMO komo;
  komo.setModel(C, true);
  komo.setTiming(2, 1, 3., 1);
  komo.add_qControlObjective({}, 1, 1e-1);

  // homing
  if(qHome.N) komo.addObjective({}, FS_qItself, {}, OT_sos, {.1}, qHome);

  // generic collisions
#if 0 //explicit enumeration -- but that's inefficient for large scenes; the iterative approach using accumulated is more effective
  if(collisionPairs.N){
    komo.addObjective({}, FS_distance, framesToNames(collisionPairs), OT_ineq, {1e2}, {-.001});
  }
#else
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1});
#endif

  arr boxSize={.06,.15,.09};

  //-- pick
  komo.addModeSwitch({1.,2.}, rai::SY_stable, {gripperName, boxName}, true);
  addBoxPickObjectives(komo, 1., pickDirection, boxName, boxSize, gripperName, palmName, tableName);

  //-- place
  komo.addModeSwitch({2.,-1.}, rai::SY_stable, {"table", boxName}, false);
  addBoxPlaceObjectives(komo, 2., placeDirection, boxName, boxSize, gripperName, palmName);

  // explicit collision avoidances
//  for(const Avoid& a:avoids){
//    komo.addObjective(a.times, FS_distance, a.frames, OT_ineq, {1e1}, {-a.dist});
//  }

//  komo.addObjective({}, FS_distance, {"r_panda_coll6", "table"}, OT_ineq, {1e2}, {});
//  komo.addObjective({}, FS_distance, {"r_panda_coll7", "table"}, OT_ineq, {1e2}, {});
//  komo.addObjective({}, FS_distance, {"r_palm", "table"}, OT_ineq, {1e2}, {});


  //-- run several times with random initialization
  bool feasible=false;
  uint trials=3;
  for(uint trial=0;trial<trials;trial++){
    //initialize with constant q0 or qTarget
    komo.reset();
    if(trial%2) komo.initWithConstant(qHome);
    else komo.initWithConstant(q0);

    //optimize
    komo.optimize(.01*trial, OptOptions().set_stopTolerance(1e-3)); //trial=0 -> no noise!

    //is feasible?
    feasible=komo.sos<50. && komo.ineq<.1 && komo.eq<.1;

    //if not feasible -> add explicit collision pairs (from proxies presently in komo.pathConfig)
    if(!feasible){
//      cout <<komo.getReport(false);
      //komo.pathConfig.reportProxies();
      StringA collisionPairs = komo.getCollisionPairs(.01);
      if(collisionPairs.N){
        komo.addObjective({}, FS_distance, collisionPairs, OT_ineq, {1e2}, {-.001});
      }
    }

    cout <<"  seq  trial " <<trial <<(feasible?" good":" FAIL") <<" -- time:" <<komo.timeTotal <<"\t sos:" <<komo.sos <<"\t ineq:" <<komo.ineq <<"\t eq:" <<komo.eq <<endl;
    if(feasible) break;
  }


  if(!feasible) return {};

  arr path = komo.getPath_qOrg();

  return path;
}

//===========================================================================

void testPnp2() {
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));

  C.addFrame("target", "table")
      ->setJoint(rai::JT_rigid)
      .setShape(rai::ST_ssBox, {.06,.15,.09,.01})
      .setRelativePosition(arr{-.3,-.2,.095});

  arr qHome = C.getJointState();
  cout <<"JOINT LIMITS:" <<C.getLimits() <<endl;

  //-- start a robot thread
  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.home(C);

  const char* gripperName="l_gripper";
  const char* palmName="l_palm";
  const char* boxName="target";
  const char* tableName="table";
  const char* arm1Name="l_panda_coll7";
  const char* arm2Name="l_panda_coll6";

  uint L=50;
  for(uint l=0;l<=L;l++){
    //-- compute keyframes
    rai::Enum<rai::ArgWord> placeDirection = random(rai::Array<rai::ArgWord>{rai::_yAxis, rai::_zAxis, rai::_yNegAxis, rai::_zNegAxis });
//    placeDirection = rai::_yNegAxis;
    cout <<"========= PLACEMENT " <<l <<": " <<placeDirection <<endl;
    C.setJointState(bot.get_q());
    arr keyframes = getPnpKeyframes(C, rai::_xAxis, placeDirection, boxName, gripperName, palmName, tableName, qHome);

    if(!keyframes.N) continue; //infeasible

    for(uint k=0;k<keyframes.d0;k++){
      arr q = bot.get_q();
//      if(k>0) CHECK_LE(maxDiff(q,keyframes[k-1]), .03, "why is the joint error so large?");
      C.setJointState(q);
      arr path;
      if(l==L) k=2;
      if(k==0){ //move to keyframes[0]
        C.attach(tableName, boxName);
        path = getStartGoalPath(C, keyframes[0], qHome, { {{.3,.7}, {palmName, boxName}, .1},
                                                          {{}, {palmName, boxName}, .0},
                                                          {{}, {arm1Name, tableName}, .0},
                                                          {{}, {arm2Name, tableName}, .0},
                                                        }, {gripperName}, true, true);
        if(!path.N) break;
      }
      if(k==1){ //move to keyframes[1]
        C.attach(gripperName, boxName);
        path = getStartGoalPath(C, keyframes[1], qHome, { {{.3,.7}, {boxName, tableName}, .05},
                                                          {{}, {boxName, tableName}, .0},
                                                          {{}, {arm1Name, tableName}, .0},
                                                          {{}, {arm2Name, tableName}, .0}
                                                        }, {gripperName}, false, false);
        if(!path.N) break;
      }
      if(k==2){ //move to home
        C.attach(tableName, boxName);
        path = getStartGoalPath(C, qHome, qHome, { {{.3,.5}, {palmName, boxName}, .1},
                                                   {{}, {palmName, boxName}, .0},
                                                   {{}, {arm1Name, tableName}, .0}
                                }, {gripperName}, false, true);
      }

      if(bot.gripperL){
        if(k==0){ bot.gripperL->open(); while(!bot.gripperL->isDone()) rai::wait(.1); }
        else if(k==1){ bot.gripperL->close(); while(!bot.gripperL->isDone()) rai::wait(.1); }
        else if(k==2){ bot.gripperL->open(); while(!bot.gripperL->isDone()) rai::wait(.1); }
      }

      //send komo as spline:
//      bot.move(path, {2.5});
      bot.moveAutoTimed(path, .003);

      while(bot.step(C));
      if(bot.keypressed=='q' || bot.keypressed==27) return;

//      if(l==0 && k==0) rai::wait();
    }
  }

}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

//  rnd.clockSeed();
  rnd.seed(0);

//  testPnp();
  testPnp2();

  cout <<"bye bye" <<endl;

  return 0;
}
