#include <Franka/controlEmulator.h>
#include <Franka/franka.h>

#include <Control/SplineCtrlFeed.h>

#include <Kin/frame.h>

#include <KOMO/komo.h>
#include <Kin/F_qFeatures.h>
#include <Kin/viewer.h>
#include <Control/LeapMPC.h>

//===========================================================================

void testLeapCtrl() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));

  //add a target in position space
  arr center = C["R_gripperCenter"]->getPosition();
  C.addFrame("target")
      ->setShape(rai::ST_marker, {.1})
      .setPosition(center + arr{+.3,.0,+.2});
  C.watch(true);


  LeapMPC leap(C,1.);
  leap.komo.addObjective({2.}, FS_positionDiff, {"R_gripperCenter", "target"}, OT_eq, {1e2});

  ofstream fil("z.dat");
  for(uint k=0;k<100;k++){
    leap.step(C);
    C.setJointState(leap.x1);
    C.watch(false);
    fil <<C.eval(FS_positionDiff, {"R_gripperCenter", "target"}).y.modRaw() <<endl;
    cout <<leap.tau <<endl;
  }
  gnuplot("plot 'z.dat' us 0:1, '' us 0:2, '' us 0:3");
  rai::wait();

  return;


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
//  FrankaThreadNew robot(ctrlRef, ctrlState, 0, franka_getJointIndices(C.get()(),'R'));
  robot.writeData = true;

  //-- define the reference feed to be a spline
  auto sp = make_shared<rai::SplineCtrlReference>();
  robot.cmd.set()->ref = sp;

  //send komo as spline:
  for(double speed=.5;speed<=3.;speed+=.5){
    sp->append(komo.getPath_qOrg(), komo.getPath_times()/speed);

    for(;;){
      C.gl()->raiseWindow();
      int key = C.watch(false,STRING("time: "<<rai::realTime() <<"\n[q or ESC to ABORT]"));
      if(key==13) break;
      if(key=='q' || key==27) return;
      C.setJointState(robot.state.get()->q);
      rai::wait(.1);
    }
  }

}

//===========================================================================

int main(int argc, char * argv[]){
  testLeapCtrl();

  return 0;
}
