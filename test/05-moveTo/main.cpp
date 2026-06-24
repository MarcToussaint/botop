#include <BotOp/simulation.h>
#include <Franka/franka.h>
#include <Franka/help.h>
#include <Algo/SplineCtrlFeed.h>
#include <Kin/viewer.h>
#include <Kin/frame.h>
#include <KOMO/komo.h>
#include <Optim/NLP_Solver.h>
#include <BotOp/bot.h>

const char *USAGE =
    "\nTest of low-level (without bot interface) of SplineCtrlReference"
    "\n";

//===========================================================================

void test_bot() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile("$RAI_PATH/scenarios/pandaSingle.g");
  C.view(true);

  //-- start a robot thread
  BotOp bot(C, rai::getParameter<bool>("real", false));

  bot.setControllerWriteData(1);

  //-- create 3 simple reference configurations
  arr q0 = bot.get_qHome();
  arr q1 = q0;
  q1(1) += .4;

  for(;;){
    LOG(0) <<"adding snake";
    bot.move((q1, q0, q1).reshape(-1, q0.N), {.5, 1., 2.});
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
  C.get_viewer()->_resetPressedKey();
  for(;;){
    if(C.view(false, STRING("time: "<<robot->state.get()->ctrlTime))=='q') break;
    C.setJointState(robot->state.get()->q);
    rai::wait(.1);
  }
  //rai::wait();
}

//===========================================================================

void test_mini() {
  rai::Configuration C;
  C.addFile("$RAI_PATH/scenarios/pandaSingle.g");
  C.view(false);
  BotOp bot(C, rai::getParameter<bool>("real", false));

  bot.setControllerWriteData(1);

  //-- create 3 simple reference configurations
  arr q0 = bot.get_qHome();
  arr q1 = q0;
  q1(1) += .1;
  cout <<q0(1) <<' ' <<q1(1) <<endl;

#if 0
  bot.move(q1.reshape(-1, q0.N), {.5});
#else
  ActionObservation obs;
  double tau_step = .05, lambda=.2;
  for(uint t=0;t<2./tau_step;t++){
    obs = bot.getActionObservation();
    arr delta = q1 - obs.qpos;
    cout <<obs.ctrlTime <<' ' <<q1(1) <<' ' <<obs.qpos(1) <<' ' <<obs.qref(1) <<' ' <<delta(1) <<endl;
    bot.stepAction(delta, obs, lambda, 1.);
    bot.sync(C,tau_step);
  }
#endif

  // bot.wait(C);
}

//===========================================================================

auto mini_IK(rai::Configuration& C, const arr& pos, const arr& qHome){
  arr q0 = C.getJointState();
  KOMO komo(C, 1, 1, 0, false);
  komo.addObjective({}, FS_jointState, {}, OT_sos, {1e-1}, q0); //cost: close to 'current state'
  komo.addObjective({}, FS_jointState, {}, OT_sos, {1e-1}, qHome); //cost: close to qHome
  komo.addObjective({}, FS_positionDiff, {"l_gripper", "target"}, OT_eq, {1e1}); //constraint: gripper position

  auto ret = rai::NLP_Solver(komo.nlp(), 0) .solve();

  return ret;
}

void test_reactive_control(){
  //-- setup a configuration
  rai::Configuration C;
  C.addFile("$RAI_PATH/scenarios/pandaSingle.g");
  arr qHome = C.getJointState();
  C.view(false);

  rai::Frame* target = C.addFrame("target", "table");
  target->setShape(rai::ST_marker, {.1});
  target->setRelativePosition({0., .3, .3});
  arr pos = target->getPosition();
  arr cen = pos;
  C.view(true);

  for(uint t=0;t<0;t++){
    rai::wait(.1);
    pos = cen + .98 * (pos-cen) + 0.02 * randn(3);
    target->setPosition(pos);

    auto ret = mini_IK(C, pos, qHome);
    cout <<*ret <<endl;
    C.setJointState(ret->x);
    C.view();
  }

  BotOp bot(C, rai::getParameter<bool>("real", false));
  ActionObservation obs;

  double tau_step = .05, lambda = .2;
  for(uint t=0;t<100;t++){
    bot.sync(C, tau_step); // #keep the workspace C sync'ed to real/sim, and idle .1 sec
    pos = cen + .98 * (pos-cen) + 0.02 * randn(3);
    target->setPosition(pos);

    auto ret = mini_IK(C, pos, qHome);
    if(ret->feasible){
//      bot.moveTo(ret->x, 5., true);
       obs = bot.getActionObservation();
       bot.stepAction(ret->x - obs.qpos, obs, lambda);
    }
  }

  rai::wait(.1);
  bot.home(C);
  rai::wait(.1);
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  cout <<USAGE <<endl;

  // test_mini();
  // test_bot();
  // test_withoutBotWrapper();
  test_reactive_control();

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::params() <<'\n';

  return 0;
}
