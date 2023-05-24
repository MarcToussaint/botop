#include <BotOp/bot.h>
#include <KOMO/komo.h>
#include <Optim/NLP_Solver.h>
#include <Kin/frame.h>

//===========================================================================

void testGrasp() {
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  C.view();

  C.addFrame("obj")
      ->setPosition({-.25,.1,.675})
      .setShape(rai::ST_ssBox, {.05,.05,.05,.005})
//      .setShape(rai::ST_sphere, {.025})
      .setColor({1,.5,0})
      .setMass(.1)
      .setContact(true);
  C.view();

  rai::Frame *way0 = C.addFrame("way0", "obj");
  rai::Frame *way1 = C.addFrame("way1", "obj");

  way0->setShape(rai::ST_marker, {.1});
  way0->setRelativePose("t(0 0 .1) d(90 0 0 1)");
  way1->setShape(rai::ST_marker, {.1});
  way1->setRelativePose("d(90 0 0 1)");

  C.view();


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


  komo.view(false, "waypoints solution");


  komo.view_close();
  arr path = komo.getPath_qOrg();


  BotOp bot(C, false);
  bot.home(C);

  bot.gripperOpen(rai::_left);
  while(!bot.gripperDone(rai::_left)){
    bot.sync(C, .1);
  }

  bot.move(path, {2., 3.});
  while( bot.getTimeToEnd()>0){
    bot.sync(C, .1);
  }

  bot.gripperCloseGrasp(rai::_left, "obj");
  while(!bot.gripperDone(rai::_left)){
    bot.sync(C, .1);
  }

  bot.home(C);


  bot.gripperOpen(rai::_left);
  while(!bot.gripperDone(rai::_left)){
    bot.sync(C, .1);
  }
}

//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  testGrasp();

  return 0;
}
