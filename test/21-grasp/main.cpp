#include <BotOp/bot.h>
#include <KOMO/komo.h>
#include <Optim/NLP_Solver.h>
#include <Kin/frame.h>
#include <KOMO/pathTools.h>

//===========================================================================

void testGrasp() {
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  C.view();

  C.addFrame("obj")
      ->setPosition({-.25,.1,.7})
      .setShape(rai::ST_ssBox, {.04,.2,.1,.005})
      .setColor({1,.5,0})
      .setMass(.1)
      .setContact(true);
  C.view();

  rai::Frame *way0 = C.addFrame("way0", "obj");
  rai::Frame *way1 = C.addFrame("way1", "obj");

  way0->setShape(rai::ST_marker, {.1});
  way0->setRelativePose("t(0 0 .2)");
  way1->setShape(rai::ST_marker, {.1});
  way1->setRelativePose("t(0 .0 .03)");

  C.view();

  arr path; //2 waypoint path
  {
    KOMO komo;
    komo.setConfig(C, true);
    komo.setTiming(2., 1, 5., 0);
    komo.addControlObjective({}, 0, 1e-0);
    komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq);
    komo.addObjective({}, FS_jointLimits, {}, OT_ineq);
    komo.addObjective({1.}, FS_poseDiff, {"l_gripper", "way0"}, OT_eq, {1e1});
    komo.addObjective({2.}, FS_poseDiff, {"l_gripper", "way1"}, OT_eq, {1e1});

    auto ret = NLP_Solver()
               .setProblem(komo.nlp())
               .setOptions( rai::OptOptions().set_stopTolerance(1e-2).set_verbose(4) )
               .solve();
    LOG(0) <<ret;

//    komo.view(false, "waypoints solution");
//    komo.view_close();

    path = komo.getPath_qOrg();
  }
  arr path_back (0, C.getJointStateDimension());
  path_back.append(path[0]); //1st waypoint
  path_back.append(C.getJointState()); //homing


  BotOp bot(C, false);
  bot.home(C);

  bot.gripperMove(rai::_left, .075, 1.);
  while(!bot.gripperDone(rai::_left)) bot.sync(C, .1);

  bot.move(path, {2., 3.});
  bot.wait(C, false, true);
  if(bot.keypressed=='q') return;

  bot.gripperMove(rai::_left, .0, .2);
  while(!bot.gripperDone(rai::_left)) bot.sync(C, .1);

  for(uint k=0;k<2;k++) bot.sync(C, .1);

  bot.move(path_back, {.1, .5}); //very fast upward motion!
  bot.wait(C, false, true);
  if(bot.keypressed=='q') return;

  bot.wait(C, true, false);

  bot.gripperMove(rai::_left, .075, 1.);
  while(!bot.gripperDone(rai::_left)) bot.sync(C, .1);

  for(uint k=0;k<10;k++) bot.sync(C, .1);
}

//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  testGrasp();

  cout <<rai::params() <<endl;

  return 0;
}
