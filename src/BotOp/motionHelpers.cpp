#include "motionHelpers.h"

#include <Kin/frame.h>
#include <Kin/F_qFeatures.h>
#include <KOMO/komo.h>
#include <KOMO/manipTools.h>

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

void getGraspLinePlane(arr& xLine, arr& yzPlane, FeatureSymbol& xyScalarProduct, FeatureSymbol& xzScalarProduct, const rai::ArgWord& dir){
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
}

//===========================================================================

void addBoxPickObjectives_botop(KOMO& komo, double time, rai::ArgWord dir, const char* boxName, const arr& boxSize, const char* gripperName, const char* palmName, const char* tableName) {
  arr xLine, yzPlane;
  FeatureSymbol xyScalarProduct=FS_none, xzScalarProduct=FS_none;
#if 1
  getGraspLinePlane(xLine, yzPlane, xyScalarProduct, xzScalarProduct, dir);
#else
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
#endif

  //position: center in inner target plane; X-specific
  komo.addObjective({time}, FS_positionRel, {gripperName, boxName}, OT_eq, xLine*1e2, {});
  komo.addObjective({time}, FS_positionRel, {gripperName, boxName}, OT_ineq, yzPlane*1e2, (boxSize/2.-.02));
  komo.addObjective({time}, FS_positionRel, {gripperName, boxName}, OT_ineq, yzPlane*(-1e2), -(boxSize/2.-.02));

  //orientation: grasp axis orthoginal to target plane; X-specific
  komo.addObjective({time-.2,time}, xyScalarProduct, {gripperName, boxName}, OT_eq, {1e2}, {});
  komo.addObjective({time-.2,time}, xzScalarProduct, {gripperName, boxName}, OT_eq, {1e2}, {});

  //no collision with palm
  komo.addObjective({time-.3,time}, FS_distance, {palmName, boxName}, OT_ineq, {1e1}, {-.001});
  //  komo.addObjective({time-.5,time}, FS_distance, {palmName, tableName}, OT_ineq, {1e1}, {-.05});

  //approach: only longitudial velocity, min distance before and at grasp
  if(komo.k_order>1) komo.addObjective({time-.3,time}, FS_positionRel, {boxName, gripperName}, OT_eq, arr{{2,3}, {1,0,0,0,1,0}}*1e2, {}, 1);
  if(komo.k_order>1) komo.addObjective({time-.5,time-.3}, FS_distance, {palmName, boxName}, OT_ineq, {1e1}, {-.1});

  //zero vel
  if(komo.k_order>1) komo.addObjective({time}, FS_qItself, {}, OT_eq, {}, {}, 1);
}

//===========================================================================

void addBoxPlaceObjectives_botop(KOMO& komo, double time, rai::ArgWord dir, const char* boxName, const arr& boxSize, const char* gripperName, const char* palmName) {
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


arr getBoxPnpKeyframes(const rai::Configuration& C, rai::ArgWord pickDirection, rai::ArgWord placeDirection, const char* boxName, const char* gripperName, const char* palmName, const char* tableName, const arr& qHome) {
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
  addBoxPlaceObjectives(komo, 2., placeDirection, boxName, boxSize, tableName, gripperName, palmName);

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
    komo.optimize(.01*trial, rai::OptOptions().set_stopTolerance(1e-3)); //trial=0 -> no noise!

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

