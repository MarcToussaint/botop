#include <BotOp/simulation.h>
#include <Franka/franka.h>
#include <Franka/help.h>
#include <Algo/SplineCtrlFeed.h>
#include <Kin/viewer.h>
#include <Kin/F_pose.h>

#include <BotOp/bot.h>

const char *USAGE =
    "\nTest of low-level (without bot interface) of SplineCtrlReference"
    "\n";

//===========================================================================

void test_bot() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));

  //-- start a robot thread
  BotOp bot(C, rai::getParameter<bool>("real", false));

  bot.gripperClose(rai::_left);

  //-- create 2 simple reference configurations
  arr q0 = bot.get_qHome();
  arr qT = q0;
  qT(1) -= .5;

  bot.move(~qT, {2.}); //in 2 sec strict
  bot.wait(C, false, true);
  bot.moveTo(q0, 1.); //using timing cost=1
  bot.wait(C, false, true);

  bot.hold(false, true);

  for(;;){
    bot.sync(C, .1);
    if(bot.keypressed=='q') break;

    arr y = C.eval(FS_position, {"l_gripper"}, {{1,3},{1,0,0}});
    bot.setCompliance(y.J(), 1.);
    cout <<y.J() * bot.get_tauExternal() <<endl;
  }

  bot.setCompliance({}, 0.);

  bot.gripperMove(rai::_left);
  bot.home(C);
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  cout <<USAGE <<endl;

  test_bot();

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::params() <<'\n';

  return 0;
}
