#include <KOMO/komo.h>
#include <BotOp/bot.h>

//===========================================================================

arr getPath(const rai::Configuration& C, int verbose=0){
  KOMO komo(C, 3., 20, 2, false);
  komo.addControlObjective({}, 2, 1.);

  komo.addObjective({1.}, FS_positionDiff, {"omnibase", "way1"}, OT_eq, {1e2});
  komo.addObjective({2.}, FS_positionDiff, {"omnibase", "way2"}, OT_eq, {1e2});
  komo.addObjective({1.,2.}, FS_vectorX, {"omnibase"}, OT_eq, {1e2}, {0,1,0});
  komo.addObjective({3.}, FS_positionDiff, {"omnibase", "way0"}, OT_eq, {1e2});
  komo.addObjective({3.}, FS_vectorX, {"omnibase"}, OT_eq, {1e2}, {1,0,0});
  komo.addObjective({3.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);

  komo.opt.verbose=verbose;
  komo.optimize();

  if(verbose>0){
    //  cout <<"TIME OPTIM: total=" <<sum(komo.getPath_times()) <<komo.getPath_times() <<endl;
    komo.plotTrajectory();
    //  komo.reportProxies();
    //  komo.checkGradients();
    komo.view(true, "result");
    while(komo.view_play(true));
  }

  return komo.getPath_qOrg();
}

//===========================================================================

void moveOmnibase(){
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/omnibase/omnibase.g"));

  C.addFrame("way0") ->setPosition({0., 0., .1});
  C.addFrame("way1") ->setPosition({.25, 0., .1});
  C.addFrame("way2") ->setPosition({0., .25, .1});
  C.view(false);

  arr q = getPath(C);

  BotOp bot(C, true);

//  bot.hold(true, false);
//  bot.wait(C, true, false); return;

  rai::wait(.1);

  bot.move(q, {9.});
  bot.wait(C);

  rai::wait();
}

//===========================================================================

int MAIN(int argc,char** argv){
  rai::initCmdLine(argc,argv);

//  rnd.clockSeed();

  moveOmnibase();

  return 0;
}

