#include <BotOp/simulation.h>
#include <Franka/franka.h>
#include <Franka/help.h>
#include <Algo/SplineCtrlFeed.h>
#include <Kin/viewer.h>

#include <BotOp/bot.h>

const char *USAGE =
    "\nTest of low-level (without bot interface) of SplineCtrlReference"
    "\n";

//===========================================================================

void test_bot() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  C.view(false);

  //-- start a robot thread
  BotOp bot(C, rai::getParameter<bool>("real", false));

  bot.setControllerWriteData(1);

  //-- create 3 simple reference configurations
  arr q0 = bot.get_qHome();
  arr q1 = q0;
  q1(1) += .4;

  for(;;){
    LOG(0) <<"adding snake";
    bot.move((q1, q0, q1).reshape(-1,q0.N), {.1, 1., 2.});
//    bot.moveTo(q1, 1., false);
    bot.wait(C);
    if(bot.keypressed=='q') break;

    LOG(0) <<"adding home";
    bot.moveTo(q0, 1., false); //using timing cost=1
    bot.wait(C);
    if(bot.keypressed=='q') break;
  }

//  LOG(0) <<"immediate stop";
//  bot.stop(C); rai::wait();

  LOG(0) <<"immediate home";
  bot.home(C);
//  bot.wait(C, false, true);
//  bot.home(C);
//  bot.wait(C, true, true);
//  if(bot.keypressed=='q') return;

//  rai::wait();
}

//===========================================================================

void test_withoutBotWrapper() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  C.view(true);

  //-- start a robot thread
  C.ensure_indexedJoints();
  std::shared_ptr<rai::RobotAbstraction> robot;
  if(rai::getParameter<bool>("real", false)){
    robot = make_shared<FrankaThread>(0, franka_getJointIndices(C,'l'));
  }else{
    robot = make_shared<BotThreadedSim>(C);
  }
  robot->writeData = 2;

  //-- create 2 simple reference configurations
  arr q0 = robot->state.get()->q;
  arr qT = q0;
  qT(1) -= .5;

  //-- define the reference feed to be a spline
  auto sp = make_shared<rai::BSplineCtrlReference>();
//  auto sp = make_shared<rai::SplineCtrlReference>();
  robot->cmd.set()->ref = sp;

  //1st motion:
  double ctrlTime = robot->state.get()->ctrlTime;
  sp->report(ctrlTime);
  sp->append((qT, q0).reshape(2,-1), arr{2., 4.}, ctrlTime);
  sp->report(ctrlTime);

  for(;;){
    if(C.view(false, STRING("time: "<<robot->state.get()->ctrlTime <<" - hit 'q' to append more"))=='q') break;
    C.setJointState(robot->state.get()->q);
    rai::wait(.1);
  }

  //2nd motion:
  ctrlTime = robot->state.get()->ctrlTime;
  sp->overwriteSmooth(~qT, {1.}, ctrlTime);
  cout <<"OVERRIDE AT t=" <<ctrlTime <<endl;
  sp->report(ctrlTime);
  sp->append(~q0, {1.}, ctrlTime);
  sp->report(ctrlTime);

  rai::wait(.1);
  C.viewer()->resetPressedKey();
  for(;;){
    if(C.view(false, STRING("time: "<<robot->state.get()->ctrlTime))=='q') break;
    C.setJointState(robot->state.get()->q);
    rai::wait(.1);
  }
  //rai::wait();
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  cout <<USAGE <<endl;

  test_bot();
  //test_withoutBotWrapper();

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::params() <<'\n';

  return 0;
}
