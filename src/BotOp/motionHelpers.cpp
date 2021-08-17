#include "motionHelpers.h"

#include <Kin/frame.h>
#include <Kin/F_qFeatures.h>
#include <KOMO/komo.h>

//===========================================================================

arr getLoopPath(rai::Configuration& C){
  //add some targets in position space
  arr center = C["r_gripper"]->getPosition();
  C.addFrame("target1")
      ->setShape(rai::ST_marker, {.1})
      .setPosition(center + arr{+.3,.0,+.2});
  C.addFrame("target2")
      ->setShape(rai::ST_marker, {.1})
      .setPosition(center + arr{+.3,.0,-.2});
  C.addFrame("target3")
      ->setShape(rai::ST_marker, {.1})
      .setPosition(center + arr{-.3,.0,+.2});
  C.addFrame("target4")
      ->setShape(rai::ST_marker, {.1})
      .setPosition(center + arr{-.3,.0,-.2});
  C.watch(false);
  arr q0 = C.getJointState();

  //compute a path
  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(5, 10, 2., 2);
  komo.add_qControlObjective({}, 2, 1.);

  komo.addObjective({1.}, FS_positionDiff, {"r_gripper", "target1"}, OT_eq, {1e2});
  komo.addObjective({2.}, FS_positionDiff, {"r_gripper", "target2"}, OT_eq, {1e2});
  komo.addObjective({3.}, FS_positionDiff, {"r_gripper", "target3"}, OT_eq, {1e2});
  komo.addObjective({4.}, FS_positionDiff, {"r_gripper", "target4"}, OT_eq, {1e2});
  komo.addObjective({5.}, make_shared<F_qItself>(C.getCtrlFramesAndScale(), true), {}, OT_eq, {1e2});
  komo.addObjective({5.}, make_shared<F_qItself>(C.getCtrlFramesAndScale()), {}, OT_eq, {1e2}, {}, 1);

  komo.optimize();
  komo.view(true);
  arr q = komo.getPath_qOrg();
  if(C["l_gripper"]){
    CHECK_EQ(q.d1, 14, "");
    //special case: copy path to other arm
    q.reshape(q.d0,2,7);
    for(uint t=0;t<q.d0;t++) q(t,0,{}) = q(t,1,{});
    q.reshape(q.d0,14);
    komo.x = q;
    komo.set_x(q);
    komo.view(true);
  }
  return q;
}

//===========================================================================

void addBoxPickObjectives(KOMO& komo, double time, rai::ArgWord dir, const char* boxName, const arr& boxSize, const char* gripperName, const char* palmName, const char* tableName) {
  arr xLine, yzPlane;
  FeatureSymbol xyScalarProduct=FS_none, xzScalarProduct=FS_none;
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

void addBoxPlaceObjectives(KOMO& komo, double time, rai::ArgWord dir, const char* boxName, const arr& boxSize, const char* gripperName, const char* palmName) {
  double relPos=0.;
  FeatureSymbol zVector = FS_none;
  arr zVectorTarget = {0.,0.,1.};
  if(dir==rai::_xAxis){
    relPos = .5*boxSize(0)+.05;
    zVector = FS_vectorX;
  } else if(dir==rai::_yAxis){
    relPos = .5*boxSize(1)+.05;
    zVector = FS_vectorY;
  } else if(dir==rai::_zAxis){
    relPos = .5*boxSize(2)+.05;
    zVector = FS_vectorZ;
  } else if(dir==rai::_xNegAxis){
    relPos = .5*boxSize(0)+.05;
    zVector = FS_vectorX;
    zVectorTarget *= -1.;
  } else if(dir==rai::_yNegAxis){
    relPos = .5*boxSize(1)+.05;
    zVector = FS_vectorY;
    zVectorTarget *= -1.;
  } else if(dir==rai::_zNegAxis){
    relPos = .5*boxSize(2)+.05;
    zVector = FS_vectorZ;
    zVectorTarget *= -1.;
  }

  //position: fixed
  komo.addObjective({time}, FS_positionDiff, {boxName, "table"}, OT_eq, {1e2}, {-.3, .1, relPos});

  //orientation: Y-up
  komo.addObjective({time-.2, time}, zVector, {boxName}, OT_eq, {1e1}, zVectorTarget);

  //retract: only longitudial velocity, min distance after grasp
  if(komo.k_order>1) komo.addObjective({time,time+.3}, FS_positionRel, {boxName, gripperName}, OT_eq, arr{{2,3}, {1,0,0,0,1,0}}*1e2, {}, 1);
  if(komo.k_order>1) komo.addObjective({time+.3,time+.5}, FS_distance, {palmName, boxName}, OT_ineq, {1e1}, {-.1});

  //zero vel
  if(komo.k_order>1) komo.addObjective({time}, FS_qItself, {}, OT_eq, {}, {}, 1);
}

//===========================================================================

arr getStartGoalPath(rai::Configuration& C, const arr& qTarget, const arr& qHome, const rai::Array<Avoid>& avoids, const char* endeff, bool endeffApproach, bool endeffRetract) {

  arr q0 = C.getJointState();
  rai::Transformation start, target; //start/target endeff poses
  if(endeff){
    start = C[endeff]->ensure_X();
    C.setJointState(qTarget);
    target = C[endeff]->ensure_X();
    C["helper"]->set_X() = target;
    C.setJointState(q0);
  }

  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(1., 32, 3., 2);
  komo.add_qControlObjective({}, 2, 1.);

  //constrain target - either endeff target or qTarget
  if(endeff){
    komo.addObjective({1.}, FS_poseDiff, {endeff, "helper"}, OT_eq, {1e0});
  }else{
    komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e0}, qTarget);
  }

  //final still
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e0}, {}, 1);

  //homing
  komo.addObjective({.4,.6}, FS_qItself, {}, OT_sos, {1.}, qHome);

  // collision avoidances
  for(const Avoid& a:avoids){
    komo.addObjective(a.times, FS_distance, a.frames, OT_ineq, {1e1}, {-a.dist});
  }

  //retract: only longitudial velocity
  if(endeffRetract){
    arr ori = ~start.rot.getArr();
    arr yz = ori({1,2});
    komo.addObjective({0.,.2}, FS_position, {endeff}, OT_eq, ori[0].reshape(1,-1)*1e2, {}, 1);
    komo.addObjective({0.,.2}, FS_angularVel, {endeff}, OT_eq, yz*1e1, {}, 1);
//    komo.addObjective({0.,.2}, FS_vectorX, {retract}, OT_eq, {1e1}, ori[0]);
//    komo.addObjective({0.,.2}, FS_vectorY, {retract}, OT_eq, {1e1}, ori[1]);
//    komo.addObjective({0.,.2}, FS_vectorZ, {retract}, OT_eq, {1e1}, ori[2]);
  }

  //approach: only longitudial velocity
  if(endeffApproach){
    arr ori = ~target.rot.getArr();
    arr yz = ori({1,2});
    komo.addObjective({.8,1.}, FS_position, {endeff}, OT_eq, ori[0].reshape(1,-1)*1e2, {}, 1);
    komo.addObjective({.8,1.}, FS_angularVel, {endeff}, OT_eq, yz*1e1, {}, 1);
//    komo.addObjective({.8,1.}, FS_vectorX, {approach}, OT_eq, {1e1}, ori[0]);
//    komo.addObjective({.8,1.}, FS_vectorY, {approach}, OT_eq, {1e1}, ori[1]);
//    komo.addObjective({.8,1.}, FS_vectorZ, {approach}, OT_eq, {1e1}, ori[2]);
  }

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
