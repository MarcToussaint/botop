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

void addBoxPickObjectives(KOMO& komo, double time, rai::ArgWord dir, const char* boxName, const arr& boxSize, const char* gripperName, const char* palmName) {
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
  komo.addObjective({time-.3,time}, FS_distance, {palmName, boxName}, OT_ineq, {1e1}, {-.005});

  //zero vel
  if(komo.k_order>1) komo.addObjective({time}, FS_qItself, {}, OT_eq, {}, {}, 1);
}

//===========================================================================

void addBoxPlaceObjectives(KOMO& komo, double time, rai::ArgWord dir, const char* boxName, arr boxSize, const char* gripperName, const char* palmName) {
  double relPos;
  FeatureSymbol zVector;
  if(dir==rai::_xAxis){
    relPos = .5*boxSize(0)+.03;
    zVector = FS_vectorX;
  } else if(dir==rai::_yAxis){
    relPos = .5*boxSize(1)+.03;
    zVector = FS_vectorY;
  } else if(dir==rai::_zAxis){
    relPos = .5*boxSize(2)+.03;
    zVector = FS_vectorZ;
  }

  //position: fixed
  komo.addObjective({time}, FS_positionDiff, {boxName, "table"}, OT_eq, {1e2}, {-.3,-.1,relPos});

  //orientation: Y-up
  komo.addObjective({time-.2, time}, zVector, {boxName}, OT_eq, {1e1}, {0., 0., 1.});

  //retract: only longitudial velocity, min distance after grasp
  if(komo.k_order>1) komo.addObjective({time,time+.3}, FS_positionRel, {boxName, gripperName}, OT_eq, arr{{2,3}, {1,0,0,0,1,0}}*1e2, {}, 1);
  if(komo.k_order>1) komo.addObjective({time+.3,time+.5}, FS_distance, {palmName, boxName}, OT_ineq, {1e1}, {-.1});

  //zero vel
  if(komo.k_order>1) komo.addObjective({time}, FS_qItself, {}, OT_eq, {}, {}, 1);
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
  FrankaThreadNew robot(0, franka_getJointIndices(C,'R'));
  robot.writeData = true;
  C.setJointState(robot.state.get()->q);

//  GripperEmulator gripper;
  FrankaGripper gripper(0);

  //-- compute a path
  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(3, 10, 3., 2);
  komo.add_qControlObjective({}, 2, 1e-1);
//  komo.add_qControlObjective({}, 1, 1e-1);

  const char* gripperName="R_gripperCenter";
  const char* palmName="R_gripper";
  const char* boxName="target";
  arr boxSize={.06,.15,.09};

  //-- pick
  komo.addModeSwitch({1.,2.}, SY_stable, {gripperName, boxName}, true);
  addBoxPickObjectives(komo, 1., rai::_xAxis, boxName, boxSize, gripperName, palmName);

  //-- carry above table
  if(komo.k_order>1) komo.addObjective({1.2,1.8}, FS_distance, {boxName, "table"}, OT_ineq, arr{1e1}, {-.05});

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
      arr times = range(0., 1., path(k).d0-1);
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

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  testPnp();

  return 0;
}
