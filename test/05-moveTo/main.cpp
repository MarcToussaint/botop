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
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));
  C.view(false);

  //-- start a robot thread
  BotOp bot(C, rai::checkParameter<bool>("real"));

  //-- create 2 simple reference configurations
  arr q0 = bot.get_qHome();
  arr qT = q0;
  qT(1) -= .5;

  bot.move(~qT, {2.}); //in 2 sec strict

  while(bot.step(C)){} //just syncs the model config C and updates display until done

  bot.moveLeap(q0, 1.); //using timing cost=1

  while(bot.step(C)){}

  rai::wait();
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
    robot = make_shared<BotSim>(C);
  }
  robot->writeData = 2;

  //-- create 2 simple reference configurations
  arr q0 = robot->state.get()->q;
  arr qT = q0;
  qT(1) -= .5;

  //-- define the reference feed to be a spline
  auto sp = make_shared<rai::CubicSplineCtrlReference>();
//  auto sp = make_shared<rai::SplineCtrlReference>();
  robot->cmd.set()->ref = sp;

  //1st motion:
  double ctrlTime = robot->state.get()->ctrlTime;
  sp->report(ctrlTime);
  sp->append((qT, q0).reshape(2,-1), zeros(2,q0.N), arr{2., 4.}, ctrlTime);
//  sp->append((qT, q0).reshape(2,-1), arr{2., 4.}, ctrlTime, true);
  sp->report(ctrlTime);

  for(;;){
    if(C.view(false, STRING("time: "<<robot->state.get()->ctrlTime <<" - hit 'q' to append more"))=='q') break;
    C.setJointState(robot->state.get()->q);
    rai::wait(.1);
  }

  //2nd motion:
  ctrlTime = robot->state.get()->ctrlTime;
  sp->moveTo(qT, 1., ctrlTime, false);
  cout <<"OVERRIDE AT t=" <<ctrlTime <<endl;
  sp->report(ctrlTime);
  sp->moveTo(q0, 1., ctrlTime, true);
  sp->report(ctrlTime);

  rai::wait(.1);
  C.gl()->resetPressedKey();
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

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::getParameters()() <<'\n';

  return 0;
}
