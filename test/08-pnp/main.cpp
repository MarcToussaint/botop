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

void testPnp() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));

  //add a target object
  rai::Frame& obj =
      C.addFrame("target", "table")
      ->setJoint(rai::JT_rigid)
      .setShape(rai::ST_ssBox, {.06,.15,.09,.01})
      .setRelativePosition(arr{-.4,-.4,.075});
  C.watch(true);
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
  komo.add_qControlObjective({}, 2, 1.);

  bool keyframesOnly=false;

  //grasp
  komo.addModeSwitch({1.,2.}, SY_stable, {"R_gripperCenter", "target"}, true);
  komo.addObjective({1.}, FS_positionDiff, {"R_gripperCenter", "target"}, OT_eq, {1e2}, {.0,.0,.03});
  komo.addObjective({.8,1.}, FS_scalarProductXY, {"R_gripperCenter", "target"}, OT_eq, {1e2}, {0.});
  komo.addObjective({.8,1.}, FS_vectorZ, {"R_gripperCenter"}, OT_eq, {1e2}, {0., 0., 1.});
  komo.addObjective({.7,1.}, FS_positionRel, {"target", "R_gripperCenter"}, OT_eq, arr{{2,3}, {1,0,0,0,1,0}}, {});
  komo.addObjective({.5,.7}, FS_positionRel, {"target", "R_gripperCenter"}, OT_ineq, arr{{1,3}, {0,0,1.}}, {0,0,-.1});

  if(!keyframesOnly){
    //slow - down - up
    komo.addObjective({1.}, FS_qItself, {}, OT_eq, {}, {}, 1);
//    komo.addObjective({1.,1.2}, FS_position, {"R_gripperCenter"}, OT_eq, {}, {0.,0.,.5}, 2, +1, +0);
  }

  //carry above table
  komo.addObjective({1.2,1.8}, FS_positionRel, {"target", "table"}, OT_ineq, arr{{1,3}, {0,0,-1.}}, {0,0,.1});

  //place
  komo.addModeSwitch({2.,-1.}, SY_stable, {"table", "target"}, false);
  komo.addObjective({2.}, FS_positionDiff, {"target", "table"}, OT_eq, {1e2}, {-.3,-.1,.08});
  komo.addObjective({1.8, 2.}, FS_vectorZ, {"R_gripperCenter"}, OT_eq, {1e2}, {0., 0., 1.});

  if(!keyframesOnly){
    //slow - down - up
    komo.addObjective({2.}, FS_qItself, {}, OT_eq, {}, {}, 1);
//    komo.addObjective({1.1,2.1}, FS_position, {"R_gripperCenter"}, OT_eq, {}, {0.,0.,.1}, 2);
  }
  robot.writeData = true;

  //retract
  komo.addObjective({2.,2.3}, FS_positionRel, {"target", "R_gripperCenter"}, OT_eq, arr{{2,3}, {1,0,0,0,1,0}}, {});
  komo.addObjective({2.3,2.5}, FS_positionRel, {"target", "R_gripperCenter"}, OT_ineq, arr{{1,3}, {0,0,1.}}, {0,0,-.1});
  komo.addObjective({3.}, FS_qItself, {}, OT_eq, {1e1}, q0);
  komo.addObjective({3.}, FS_qItself, {}, OT_eq, {1e1}, {}, 1);

  komo.optimize();
  komo.view(true);
  while(komo.view_play(true));

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
    if(k==0) gripper.open();
    if(k==1) gripper.close();
    if(k==2) gripper.open();

    //send komo as spline:
    {
      arr times = range(0., 2., path(k).d0-1);
      times += times(1);
      double ctrlTime = robot.state.get()->time;
      sp->append(path(k), times, ctrlTime, true);
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

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  testPnp();

  return 0;
}
