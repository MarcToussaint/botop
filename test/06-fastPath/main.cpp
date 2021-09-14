#include <KOMO/komo.h>
#include <Kin/F_qFeatures.h>
#include <Core/thread.h>

#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>

//===========================================================================

void testFastPath() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));


#if 0
  //-- add some targets in position space
  arr center = C["l_gripper"]->getPosition();
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

  //-- compute a path
  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(5, 10, 2., 2);
  komo.add_qControlObjective({}, 2, 1.);

  komo.addObjective({1.}, FS_positionDiff, {"l_gripper", "target1"}, OT_eq, {1e2});
  komo.addObjective({2.}, FS_positionDiff, {"l_gripper", "target2"}, OT_eq, {1e2});
  komo.addObjective({3.}, FS_positionDiff, {"l_gripper", "target3"}, OT_eq, {1e2});
  komo.addObjective({4.}, FS_positionDiff, {"l_gripper", "target4"}, OT_eq, {1e2});
  komo.addObjective({5.}, make_shared<F_qItself>(C.getCtrlFramesAndScale(), true), {}, OT_eq, {1e2});
  komo.addObjective({5.}, make_shared<F_qItself>(C.getCtrlFramesAndScale()), {}, OT_eq, {1e2}, {}, 1);

  komo.optimize();

  komo.view(true);
  arr path = komo.getPath_qOrg();
#else
  arr path = getLoopPath(C);
#endif

  //-- start a robot thread
  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.home(C);
//  bot.setControllerWriteData(1);

  rai::wait(1.);
  rai::Frame *optiFrameR = C["ot_r_panda_gripper"];
  rai::Frame *optiFrameL = C["ot_l_panda_gripper"];
//  rai::Frame *optiTable = C["ot_table"];

  ofstream fil("z.dat");
  //-- send path as spline:
  for(double speed=1.;speed<=5.;speed+=.5){
    bot.move(path, ARR(30.)/speed);

    Metronome tic(.01);
    while(bot.step(C, -1.)){
      fil <<rai::realTime() <<" q " <<C.getJointState() <<" poseL " <<optiFrameL->get_X() <<" poseR " <<optiFrameR->get_X() <<endl; // <<" poseTable " <<optiTable->get_X() <<endl;
      tic.waitForTic();
    }
    if(bot.keypressed=='q' || bot.keypressed==27) return;
  }
  fil.close();

}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  testFastPath();

  return 0;
}
