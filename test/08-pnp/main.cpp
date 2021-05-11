#include <Franka/controlEmulator.h>
#include <Franka/franka.h>
#include <Franka/gripper.h>
#include <Franka/help.h>

#include <Algo/SplineCtrlFeed.h>

#include <Kin/frame.h>

#include <KOMO/komo.h>
#include <Kin/F_qFeatures.h>
#include <Kin/viewer.h>
#include <Control/LeapMPC.h>

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
  komo.addObjective({time-.3,time}, FS_distance, {palmName, boxName}, OT_ineq, {1e1}, {-.006});
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
  komo.addObjective({time}, FS_positionDiff, {boxName, "table"}, OT_eq, {1e2}, {-.3, -.2, relPos});

  //orientation: Y-up
  komo.addObjective({time-.2, time}, zVector, {boxName}, OT_eq, {1e1}, zVectorTarget);

  //retract: only longitudial velocity, min distance after grasp
  if(komo.k_order>1) komo.addObjective({time,time+.3}, FS_positionRel, {boxName, gripperName}, OT_eq, arr{{2,3}, {1,0,0,0,1,0}}*1e2, {}, 1);
  if(komo.k_order>1) komo.addObjective({time+.3,time+.5}, FS_distance, {palmName, boxName}, OT_ineq, {1e1}, {-.1});

  //zero vel
  if(komo.k_order>1) komo.addObjective({time}, FS_qItself, {}, OT_eq, {}, {}, 1);
}

//===========================================================================

arr getStartGoalPath(rai::Configuration& C, const arr& target_q, const char* approach=0, const char* retract=0, const rai::Array<Avoid>& avoids={}) {

  arr q0 = C.getJointState();

  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(1., 16, 3., 2);
  komo.add_qControlObjective({}, 2, 1.);

  //retract: only longitudial velocity
  if(retract){
    arr ori = ~C[retract]->getRotationMatrix();
    komo.addObjective({0.,.2}, FS_position, {retract}, OT_eq, ori[0].reshape(1,-1)*1e2, {}, 1);
    komo.addObjective({0.,.2}, FS_vectorX, {retract}, OT_eq, {1e1}, ori[0]);
    komo.addObjective({0.,.2}, FS_vectorY, {retract}, OT_eq, {1e1}, ori[1]);
    komo.addObjective({0.,.2}, FS_vectorZ, {retract}, OT_eq, {1e1}, ori[2]);
  }

  C.setJointState(target_q);

  //approach: only longitudial velocity
  if(approach){
    arr ori = ~C[approach]->getRotationMatrix();
    komo.addObjective({.8,1.}, FS_position, {approach}, OT_eq, ori[0].reshape(1,-1)*1e2, {}, 1);
    komo.addObjective({.8,1.}, FS_vectorX, {approach}, OT_eq, {1e1}, ori[0]);
    komo.addObjective({.8,1.}, FS_vectorY, {approach}, OT_eq, {1e1}, ori[1]);
    komo.addObjective({.8,1.}, FS_vectorZ, {approach}, OT_eq, {1e1}, ori[2]);
  }

  // collision avoidances
  for(const Avoid& a:avoids){
    komo.addObjective(a.times, FS_distance, a.frames, OT_ineq, {1e1}, {-a.dist});
  }

  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e0}, target_q);
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e0}, {}, 1);

//  komo.initWithWaypoints({target_q});
//  komo.initWithConstant(target_q);
  komo.optimize(0.);

  arr path = komo.getPath_qOrg();
  path[path.d0-1] = target_q; //overwrite last config
//  arr times = komo.getPath_times();

  C.setJointState(q0);

//  while(komo.view_play(true));
  if(komo.sos>30. || komo.ineq>.2 || komo.eq>.2){
    FILE("z.path") <<path <<endl;
    cout <<komo.getReport(true) <<endl;
    LOG(-2) <<"WARNING!";
    while(komo.view_play(true));

    komo.reset();
    komo.initWithConstant(q0);
    komo.animateOptimization=4;
    komo.verbose=8;
    komo.optimize();
    if(komo.sos>30. || komo.ineq>.2 || komo.eq>.2){
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
  arr q0 = C.getJointState();

  //-- start a robot thread
  C.ensure_indexedJoints();
//  ControlEmulator robot(C, {});
//  GripperEmulator gripper;
  FrankaThreadNew robot(0, franka_getJointIndices(C,'R'));
  FrankaGripper gripper(0);

  robot.writeData = true;
  C.setJointState(robot.state.get()->q);
  C.watch(false);


  //-- compute a path
  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(3, 10, 3., 2);
  komo.add_qControlObjective({}, 2, 1e-1);
//  komo.add_qControlObjective({}, 1, 1e-1);

  const char* gripperName="R_gripperCenter";
  const char* palmName="R_gripper";
  const char* boxName="target";
  const char* tableName="table";
  arr boxSize={.06,.15,.09};

  //-- pick
  komo.addModeSwitch({1.,2.}, SY_stable, {gripperName, boxName}, true);
  addBoxPickObjectives(komo, 1., rai::_xAxis, boxName, boxSize, gripperName, palmName, tableName);

  //-- carry above table
  if(komo.k_order>1) komo.addObjective({1.2,1.8}, FS_distance, {boxName, tableName}, OT_ineq, arr{1e1}, {-.05});

  //-- place
  komo.addModeSwitch({2.,-1.}, SY_stable, {"table", boxName}, false);
  addBoxPlaceObjectives(komo, 2., rai::_yAxis, boxName, boxSize, gripperName, palmName);

  //-- home
  komo.addObjective({3.}, FS_qItself, {}, OT_eq, {1e1}, q0);
  if(komo.k_order>1) komo.addObjective({3.}, FS_qItself, {}, OT_eq, {1e1}, {}, 1);


  komo.optimize();
  komo.getReport(true);
  komo.view(true);
  while(komo.view_play(true, .5));

  //-- define the reference feed to be a spline
  auto sp = make_shared<rai::SplineCtrlReference>();
  robot.cmd.set()->ref = sp;


  //get KOMO parts
  arr fullPath = komo.getPath_qOrg();
  arrA path(komo.T/komo.stepsPerPhase);
  for(uint k=0;k<path.N;k++){
    path(k) = fullPath({k*komo.stepsPerPhase,(k+1)*komo.stepsPerPhase-1});
  }

  for(uint k=0;k<path.N;k++){
//    rai::wait();
    if(k==0){ gripper.open(); rai::wait(.3); }
    if(k==1){ gripper.close(); rai::wait(.5); }
    if(k==2){ gripper.open(); rai::wait(.3); }

    //send komo as spline:
    {
      arr times = range(0., 4., path(k).d0-1);
      if(times.N==1) times=2.;
      else times += times(1);
      double ctrlTime = robot.state.get()->time;
      sp->append(path(k), times, ctrlTime, true);
    }

    for(;;){
      C.gl()->raiseWindow();
      double ctrlTime = robot.state.get()->time;
      int key = C.watch(false,STRING("time: "<<ctrlTime <<"\n[q or ESC to ABORT]"));
      //if(key==13) break;
      if(key=='q' || key==27) return;
      if(ctrlTime+.5>sp->getEndTime()) break;
      C.setJointState(robot.state.get()->q);
      rai::wait(.1);
    }
  }

  rai::wait(1.);

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
  komo.addObjective({}, FS_qItself, {}, OT_sos, {1e-1}, qHome);

  arr boxSize={.06,.15,.09};

  //-- pick
  komo.addModeSwitch({1.,2.}, SY_stable, {gripperName, boxName}, true);
  addBoxPickObjectives(komo, 1., pickDirection, boxName, boxSize, gripperName, palmName, tableName);

  //-- place
  komo.addModeSwitch({2.,-1.}, SY_stable, {"table", boxName}, false);
  addBoxPlaceObjectives(komo, 2., placeDirection, boxName, boxSize, gripperName, palmName);

  komo.addObjective({}, FS_distance, {"R_panda_coll6", "table"}, OT_ineq, {1e2}, {});
  komo.addObjective({}, FS_distance, {"R_panda_coll7", "table"}, OT_ineq, {1e2}, {});
  komo.addObjective({}, FS_distance, {"R_panda_coll_hand", "table"}, OT_ineq, {1e2}, {});

  //-- home
//  komo.addObjective({3.}, FS_qItself, {}, OT_eq, {1e1}, q0);

  komo.optimize();
  komo.getReport(true);
  //  while(komo.view_play(true, .5));

  if(komo.getConstraintViolations()>.1){
    LOG(-1) <<"INFEASIBLE";
    komo.view(true);
    komo.reset();
    komo.initWithConstant(qHome);
    komo.optimize();
    if(komo.getConstraintViolations()>.1){
      LOG(-1) <<"INFEASIBLE";
      komo.view(true);
//      komo.reset();
//      komo.initWithConstant(q0);
//      komo.animateOptimization=4;
//      komo.optimize();
      return {};
    }else{
      LOG(-1) <<"FEASIBLE";
      komo.view(true);
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
      .setRelativePosition(arr{-.4,-.4,.075});
  arr q0 = C.getJointState();
  cout <<"JOINT LIMITS:" <<C.getLimits() <<endl;

  C.ensure_indexedJoints();
#if 1 //SIM
  ControlEmulator robot(C, {});
  GripperEmulator gripper;
#else //REAL
  FrankaThreadNew robot(0, franka_getJointIndices(C,'R'));
  FrankaGripper gripper(0);
#endif
  robot.writeData = true;

  C.setJointState(robot.state.get()->q);
  C.watch(false);

  //-- define the reference feed to be a spline
  auto sp = make_shared<rai::SplineCtrlReference>();
  robot.cmd.set()->ref = sp;

  const char* gripperName="R_gripperCenter";
  const char* palmName="R_gripper";
  const char* boxName="target";
  const char* tableName="table";
  const char* arm1Name="R_panda_coll7";
  const char* arm2Name="R_panda_coll6";

  uint L=20;
  for(uint l=0;l<=L;l++){
    //-- compute keyframes
    rai::Enum<rai::ArgWord> placeDirection = random(rai::Array<rai::ArgWord>{rai::_yAxis, rai::_zAxis, rai::_yNegAxis, rai::_zNegAxis });
//    placeDirection = rai::_yNegAxis;
    cout <<"PLACING: " <<placeDirection <<endl;
    arr qNow = C.getJointState();
    arr keyframes = getPnpKeyframes(C, rai::_xAxis, placeDirection, boxName, gripperName, palmName, tableName, q0);

    if(!keyframes.N) continue; //infeasible

    cout <<C.getJointState() <<endl <<qNow <<endl <<keyframes <<endl;

    for(uint k=0;k<keyframes.d0;k++){
      arr q = robot.state.get()->q;
      if(k>0) CHECK_LE(maxDiff(q,keyframes[k-1]), 1e-6, "");
      C.setJointState(q);
      arr path;
      if(l==L) k=2;
      if(k==0){
        C.attach(tableName, boxName);
        path = getStartGoalPath(C, keyframes[0], gripperName, gripperName, {{{.3,.7}, {palmName, boxName}, .1},
                                                                            {{.8,1.}, {palmName, boxName}, .005},
                                                                            {{}, {arm1Name, tableName}, .0},
                                                                            {{}, {arm2Name, tableName}, .0},
                                                                           });
      }
      if(k==1){
        C.attach(gripperName, boxName);
        path = getStartGoalPath(C, keyframes[1], 0, 0, { {{.3,.7}, {boxName, tableName}, .05},
                                                         {{}, {boxName, tableName}, .0},
                                                         {{}, {arm1Name, tableName}, .0},
                                                         {{}, {arm2Name, tableName}, .0}
                                                       });
      }
      if(k==2){
        C.attach(tableName, boxName);
        path = getStartGoalPath(C, q0, 0, gripperName, {{{.3,.5}, {palmName, boxName}, .1},
                                                        {{}, {arm1Name, tableName}, .0}});
      }

      if(k==0){ gripper.open(); rai::wait(.3); }
      else if(k==1){ gripper.close(); rai::wait(.5); }
      else if(k==2){ gripper.open(); rai::wait(.3); }

      //send komo as spline:
      {
        arr times = range(0., 2., path.d0-1);
        if(times.N==1) times=2.;
        else times += times(1);
        double ctrlTime = robot.state.get()->time;
        sp->append(path, times, ctrlTime, true);
      }

      for(;;){
        C.gl()->raiseWindow();
        double ctrlTime = robot.state.get()->time;
        int key = C.watch(false,STRING("time: "<<ctrlTime <<"\n[q or ESC to ABORT]"));
        //if(key==13) break;
        if(key=='q' || key==27) return;
        if(ctrlTime>sp->getEndTime()) break;
        C.setJointState(robot.state.get()->q);
        rai::wait(.1);
      }
    }
  }

  rai::wait(1.);

}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

//  rnd.clockSeed();

//  testPnp();
  testPnp2();

  return 0;
}
