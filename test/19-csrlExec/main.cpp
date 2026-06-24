#include <BotOp/bot.h>
#include <KOMO/komo.h>
#include <Optim/NLP_Solver.h>
#include <Kin/frame.h>
#include <KOMO/pathTools.h>

//===========================================================================

void test() {
  rai::Configuration C;
  C.addFile(rai::raiPath("../../26-sgrl/scenes/double_sphere.yml"));
  C.view();

  arr q0 = C.getJointState();

  BotOp bot(C, false);
  // bot.home(C);

  // arr q = q0;
  // q(0) += .1;
  // bot.moveTo(q);
  // bot.gripperMove(rai::_left, .075, 1.);
  // while(!bot.gripperDone(rai::_left)) bot.sync(C, .1);

  // bot.move(path, {2., 3.});
  // bot.wait(C, false, true);
  // if(bot.keypressed=='q') return;

  // bot.gripperMove(rai::_left, .0, .2);
  // while(!bot.gripperDone(rai::_left)) bot.sync(C, .1);

  // for(uint k=0;k<2;k++) bot.sync(C, .1);

  // bot.move(path_back, {.1, .5}); //very fast upward motion!
  // bot.wait(C, false, true);
  // if(bot.keypressed=='q') return;

  bot.wait(C, true, false);
}

//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  test();

  cout <<rai::params() <<endl;

  return 0;
}
