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

//===========================================================================

struct Avoid{
  arr times;
  StringA frames;
  double dist;
};

//===========================================================================

void addBoxPickObjectives(KOMO& komo, double time, rai::ArgWord dir, const char* boxName, const arr& boxSize, const char* gripperName, const char* palmName, const char* tableName) {
  arr xLine, yzPlane;
  FeatureSymbol xyScalarProduct, xzScalarProduct;
  if(dir==rai::_xAxis){
    xLine = {{1,3},{1,0,0}};
    yzPlane = {{2,3},{0,1,0,0,0,1}};
    xyScalarProduct = FS_scalarProductXY;
    xzScalarProduct = FS_scalarProductXZ;
  } else if(dir==rai::_yAxis){
    xLine = {{1,3},{0,1,0}};
    yzPlane = {{2,3},{1,0,0,0,0,1}};
    xyScalarProduct = FS_scalarProductXX;
    xzScalarProduct = FS_scalarProductXZ;
  } else if(dir==rai::_zAxis){
    xLine = {{1,3},{0,0,1}};
    yzPlane = {{2,3},{1,0,0,0,1,0}};
    xyScalarProduct = FS_scalarProductXX;
    xzScalarProduct = FS_scalarProductXY;
  }

  //position: center in inner target plane; X-specific
  komo.addObjective({time}, FS_positionRel, {gripperName, boxName}, OT_eq, xLine*1e2, {});
  komo.addObjective({time}, FS_positionRel, {gripperName, boxName}, OT_ineq, yzPlane*1e2, (boxSize/2.-.02));
  komo.addObjective({time}, FS_positionRel, {gripperName, boxName}, OT_ineq, yzPlane*(-1e2), -(boxSize/2.-.02));

  //orientation: grasp axis orthoginal to target plane; X-specific
  komo.addObjective({time-.2,time}, xyScalarProduct, {gripperName, boxName}, OT_eq, {1e2}, {});
  komo.addObjective({time-.2,time}, xzScalarProduct, {gripperName, boxName}, OT_eq, {1e2}, {});

  //approach: only longitudial velocity, min distance before and at grasp
  if(komo.k_order>1) komo.addObjective({time-.3,time}, FS_positionRel, {boxName, gripperName}, OT_eq, arr{{2,3}, {1,0,0,0,1,0}}*1e2, {}, 1);
  if(komo.k_order>1) komo.addObjective({time-.5,time-.3}, FS_distance, {palmName, boxName}, OT_ineq, {1e1}, {-.1});
  komo.addObjective({time-.3,time}, FS_distance, {palmName, boxName}, OT_ineq, {1e1}, {-.001});
//  komo.addObjective({time-.5,time}, FS_distance, {palmName, tableName}, OT_ineq, {1e1}, {-.05});

  //zero vel
  if(komo.k_order>1) komo.addObjective({time}, FS_qItself, {}, OT_eq, {}, {}, 1);
}

//===========================================================================

void addBoxPlaceObjectives(KOMO& komo, double time, rai::ArgWord dir, const char* boxName, arr boxSize, const char* gripperName, const char* palmName) {
  double relPos;
  FeatureSymbol zVector;
  arr zVectorTarget = {0.,0.,1.};
  if(dir==rai::_xAxis){
    relPos = .5*boxSize(0)+.03;
    zVector = FS_vectorX;
  } else if(dir==rai::_yAxis){
    relPos = .5*boxSize(1)+.03;
    zVector = FS_vectorY;
  } else if(dir==rai::_zAxis){
    relPos = .5*boxSize(2)+.03;
    zVector = FS_vectorZ;
  } else if(dir==rai::_xNegAxis){
    relPos = .5*boxSize(0)+.03;
    zVector = FS_vectorX;
    zVectorTarget *= -1.;
  } else if(dir==rai::_yNegAxis){
    relPos = .5*boxSize(1)+.03;
    zVector = FS_vectorY;
    zVectorTarget *= -1.;
  } else if(dir==rai::_zNegAxis){
    relPos = .5*boxSize(2)+.03;
    zVector = FS_vectorZ;
    zVectorTarget *= -1.;
  }

  //position: fixed
  komo.addObjective({time}, FS_positionDiff, {boxName, "table"}, OT_eq, {1e2}, {-.3, .0, relPos});

  //orientation: Y-up
  komo.addObjective({time-.2, time}, zVector, {boxName}, OT_eq, {1e1}, zVectorTarget);

  //retract: only longitudial velocity, min distance after grasp
  if(komo.k_order>1) komo.addObjective({time,time+.3}, FS_positionRel, {boxName, gripperName}, OT_eq, arr{{2,3}, {1,0,0,0,1,0}}*1e2, {}, 1);
  if(komo.k_order>1) komo.addObjective({time+.3,time+.5}, FS_distance, {palmName, boxName}, OT_ineq, {1e1}, {-.1});

  //zero vel
  if(komo.k_order>1) komo.addObjective({time}, FS_qItself, {}, OT_eq, {}, {}, 1);
}

//===========================================================================

arr getStartGoalPath(rai::Configuration& C, const char* endeff, const arr& target_q, bool approach, bool retract, const rai::Array<Avoid>& avoids, const arr& qHome) {

  arr q0 = C.getJointState();
  rai::Transformation start = C[endeff]->ensure_X();
  C.setJointState(target_q);
  rai::Transformation target = C[endeff]->ensure_X();
  C["helper"]->set_X() = target;
  C.setJointState(q0);

  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(1., 32, 3., 2);
  komo.add_qControlObjective({}, 2, 1.);
  komo.addObjective({.4,.6}, FS_qItself, {}, OT_sos, {1.}, qHome);

  //retract: only longitudial velocity
  if(retract){
    arr ori = ~start.rot.getArr();
    arr yz = ori({1,2});
    komo.addObjective({0.,.2}, FS_position, {endeff}, OT_eq, ori[0].reshape(1,-1)*1e2, {}, 1);
    komo.addObjective({0.,.2}, FS_angularVel, {endeff}, OT_eq, yz*1e1, {}, 1);
//    komo.addObjective({0.,.2}, FS_vectorX, {retract}, OT_eq, {1e1}, ori[0]);
//    komo.addObjective({0.,.2}, FS_vectorY, {retract}, OT_eq, {1e1}, ori[1]);
//    komo.addObjective({0.,.2}, FS_vectorZ, {retract}, OT_eq, {1e1}, ori[2]);
  }


  //approach: only longitudial velocity
  if(approach){
    arr ori = ~target.rot.getArr();
    arr yz = ori({1,2});
    komo.addObjective({.8,1.}, FS_position, {endeff}, OT_eq, ori[0].reshape(1,-1)*1e2, {}, 1);
    komo.addObjective({.8,1.}, FS_angularVel, {endeff}, OT_eq, yz*1e1, {}, 1);
//    komo.addObjective({.8,1.}, FS_vectorX, {approach}, OT_eq, {1e1}, ori[0]);
//    komo.addObjective({.8,1.}, FS_vectorY, {approach}, OT_eq, {1e1}, ori[1]);
//    komo.addObjective({.8,1.}, FS_vectorZ, {approach}, OT_eq, {1e1}, ori[2]);
  }

  // collision avoidances
  for(const Avoid& a:avoids){
    komo.addObjective(a.times, FS_distance, a.frames, OT_ineq, {1e1}, {-a.dist});
  }

//  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e0}, target_q);
  komo.addObjective({1.}, FS_poseDiff, {endeff, "helper"}, OT_eq, {1e0});
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e0}, {}, 1);

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

    komo.reset();
//    komo.initWithWaypoints({target_q});
    komo.initWithConstant(q0);
//    komo.animateOptimization=4;
//    komo.verbose=8;
    komo.optimize();
    if(komo.sos>50. || komo.ineq>.2 || komo.eq>.2){
      LOG(-1) <<"INFEASIBLE";
      while(komo.view_play(true));
      return {};
    }else{
      LOG(-1) <<"FEASIBLE";
      komo.view(true);
    }
  }

  return path;
}

//===========================================================================

void testPnp() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));

  //add a target object
  C.addFrame("target", "table")
      ->setJoint(rai::JT_rigid)
      .setShape(rai::ST_ssBox, {.06,.15,.09,.01})
      .setRelativePosition(arr{-.4,-.4,.075});
  arr qHome = C.getJointState();

  //-- start a robot thread
  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.home(C);

  const char* gripperName="R_gripper";
  const char* palmName="R_panda_coll_hand";
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
    if(k==0){ bot.gripper->open(); rai::wait(.3); }
    if(k==1){ bot.gripper->close(); rai::wait(.5); }
    if(k==2){ bot.gripper->open(); rai::wait(.3); }

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
  komo.setModel(C, false);
  komo.setTiming(2, 1, 3., 1);
  //  komo.add_qControlObjective({}, 2, 1e-1);
  komo.add_qControlObjective({}, 1, 1e-1);
  komo.addObjective({}, FS_qItself, {}, OT_sos, {1.}, qHome);

  arr boxSize={.06,.15,.09};

  //-- pick
  komo.addModeSwitch({1.,2.}, rai::SY_stable, {gripperName, boxName}, true);
  addBoxPickObjectives(komo, 1., pickDirection, boxName, boxSize, gripperName, palmName, tableName);

  //-- place
  komo.addModeSwitch({2.,-1.}, rai::SY_stable, {"table", boxName}, false);
  addBoxPlaceObjectives(komo, 2., placeDirection, boxName, boxSize, gripperName, palmName);

  komo.addObjective({}, FS_distance, {"R_panda_coll6", "table"}, OT_ineq, {1e2}, {});
  komo.addObjective({}, FS_distance, {"R_panda_coll7", "table"}, OT_ineq, {1e2}, {});
  komo.addObjective({}, FS_distance, {"R_panda_coll_hand", "table"}, OT_ineq, {1e2}, {});

  //-- home
//  komo.addObjective({3.}, FS_qItself, {}, OT_eq, {1e1}, q0);

  komo.optimize();
  cout <<"  seq  -- time:" <<komo.timeTotal <<"\t sos:" <<komo.sos <<"\t ineq:" <<komo.ineq <<"\t eq:" <<komo.eq <<endl;
//  komo.getReport(true);
//  cout <<q0 <<endl <<komo.getPath_qOrg() <<endl;
//  while(komo.view_play(true, .5));

  if(komo.getConstraintViolations()>.1){
    LOG(-1) <<"INFEASIBLE";
    komo.view(false);
    komo.reset();
    komo.initWithConstant(qHome);
    komo.optimize();
    if(komo.getConstraintViolations()>.1){
      cout <<komo.getReport(true);
      LOG(-1) <<"INFEASIBLE";
      komo.view(true);
//      komo.reset();
//      komo.initWithConstant(q0);
//      komo.animateOptimization=4;
//      komo.optimize();
      return {};
    }else{
      LOG(-1) <<"FEASIBLE";
      komo.view(false);
    }
  }

  return komo.getPath_qOrg();
}

//===========================================================================

void testPnp2() {
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));

  C.addFrame("target", "table")
      ->setJoint(rai::JT_rigid)
      .setShape(rai::ST_ssBox, {.06,.15,.09,.01})
      .setRelativePosition(arr{-.4,-.2,.075});

  C.addFrame("helper")
      ->setShape(rai::ST_marker, {.5})
      .setColor({1.,1.,0,.5});

  arr qHome = C.getJointState();
  cout <<"JOINT LIMITS:" <<C.getLimits() <<endl;

  //-- start a robot thread
  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.home(C);

  const char* gripperName="R_gripper";
  const char* palmName="R_panda_coll_hand";
  const char* boxName="target";
  const char* tableName="table";
  const char* arm1Name="R_panda_coll7";
  const char* arm2Name="R_panda_coll6";

  uint L=50;
  for(uint l=0;l<=L;l++){
    //-- compute keyframes
    rai::Enum<rai::ArgWord> placeDirection = random(rai::Array<rai::ArgWord>{rai::_yAxis, rai::_zAxis, rai::_yNegAxis, rai::_zNegAxis });
//    placeDirection = rai::_yNegAxis;
    cout <<"PLACING: " <<placeDirection <<endl;
    C.setJointState(bot.get_q());
    arr keyframes = getPnpKeyframes(C, rai::_xAxis, placeDirection, boxName, gripperName, palmName, tableName, qHome);

    if(!keyframes.N) continue; //infeasible

    for(uint k=0;k<keyframes.d0;k++){
      arr q = bot.get_q();
//      if(k>0) CHECK_LE(maxDiff(q,keyframes[k-1]), .03, "why is the joint error so large?");
      C.setJointState(q);
      arr path;
      if(l==L) k=2;
      if(k==0){
        C.attach(tableName, boxName);
        path = getStartGoalPath(C, gripperName, keyframes[0], true, true, { {{.3,.7}, {palmName, boxName}, .1},
                                                                            {{}, {palmName, boxName}, .0},
                                                                            {{}, {arm1Name, tableName}, .0},
                                                                            {{}, {arm2Name, tableName}, .0},
                                                                          }, qHome);
        if(!path.N) break;
      }
      if(k==1){
        C.attach(gripperName, boxName);
        path = getStartGoalPath(C, gripperName, keyframes[1], false, false, { {{.3,.7}, {boxName, tableName}, .05},
                                                                              {{}, {boxName, tableName}, .0},
                                                                              {{}, {arm1Name, tableName}, .0},
                                                                              {{}, {arm2Name, tableName}, .0}
                                                                            }, qHome);
        if(!path.N) return;
      }
      if(k==2){
        C.attach(tableName, boxName);
        path = getStartGoalPath(C, gripperName, qHome, false, true, { {{.3,.5}, {palmName, boxName}, .1},
                                                                      {{}, {palmName, boxName}, .0},
                                                                      {{}, {arm1Name, tableName}, .0}
                                }, qHome);
      }

      if(k==0){ bot.gripper->open(); rai::wait(.3); }
      else if(k==1){ bot.gripper->close(100.); rai::wait(.5); }
      else if(k==2){ bot.gripper->open(); rai::wait(.3); }

      //send komo as spline:
      bot.move(path, {2.5});

      while(bot.step(C));
      if(bot.keypressed=='q' || bot.keypressed==27) return;
    }
  }

}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

//  rnd.clockSeed();

//  testPnp();
  testPnp2();

  cout <<"bye bye" <<endl;

  return 0;
}
