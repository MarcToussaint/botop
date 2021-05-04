#include <Franka/controlEmulator.h>
#include <Franka/franka.h>
#include <Franka/help.h>

#include <Algo/SplineCtrlFeed.h>

#include <Kin/frame.h>

#include <KOMO/komo.h>
#include <Kin/F_qFeatures.h>
#include <Kin/viewer.h>

//===========================================================================

void testFastPath() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));

  //add some targets in position space
  arr center = C["R_gripperCenter"]->getPosition();
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
  C.watch(true);
  arr q0 = C.getJointState();

  //compute a path
  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(5, 10, 2., 2);
  komo.add_qControlObjective({}, 2, 1.);

  komo.addObjective({1.}, FS_positionDiff, {"R_gripperCenter", "target1"}, OT_eq, {1e2});
  komo.addObjective({2.}, FS_positionDiff, {"R_gripperCenter", "target2"}, OT_eq, {1e2});
  komo.addObjective({3.}, FS_positionDiff, {"R_gripperCenter", "target3"}, OT_eq, {1e2});
  komo.addObjective({4.}, FS_positionDiff, {"R_gripperCenter", "target4"}, OT_eq, {1e2});
  komo.addObjective({5.}, make_shared<F_qItself>(C.getCtrlFramesAndScale(), true), {}, OT_eq, {1e2});
  komo.addObjective({5.}, make_shared<F_qItself>(C.getCtrlFramesAndScale()), {}, OT_eq, {1e2}, {}, 1);

  komo.optimize();

  komo.view(true);

  //-- start a robot thread
  ControlEmulator robot(C, {});
//  FrankaThreadNew robot(0, franka_getJointIndices(C,'R'));
  robot.writeData = true;

  //-- define the reference feed to be a spline
  auto sp = make_shared<rai::SplineCtrlReference>();
  robot.cmd.set()->ref = sp;

  //first move slowly to home!
  double ctrlTime = robot.state.get()->time;
  sp->moveTo(q0, 2., ctrlTime, true);

  //send komo as spline:
  for(double speed=1.;speed<=5.;speed+=.5){
    ctrlTime = robot.state.get()->time;
    sp->append(komo.getPath_qOrg(), komo.getPath_times()/speed, ctrlTime, true);

    for(;;){
      C.gl()->raiseWindow();
      int key = C.watch(false,STRING("time: "<<robot.state.get()->time <<"\n[q or ESC to ABORT]"));
      if(key==13) break;
      if(key=='q' || key==27) return;
      C.setJointState(robot.state.get()->q);
      rai::wait(.1);
    }
  }

}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  testFastPath();

  return 0;
}
