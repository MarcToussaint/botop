#include <Franka/controlEmulator.h>
#include <Franka/franka.h>
#include <Franka/help.h>

#include <Algo/SplineCtrlFeed.h>

#include <Kin/frame.h>

#include <KOMO/komo.h>
#include <Kin/F_qFeatures.h>
#include <Kin/viewer.h>
#include <Control/LeapMPC.h>

#include <BotOp/bot.h>

//===========================================================================

void testLeapCtrl() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));

  //add a target in position space
  arr center = C["R_gripper"]->getPosition();
  rai::Frame& target =
      C.addFrame("target")
      ->setShape(rai::ST_marker, {.1})
      .setPosition(center + arr{+.3,.0,+.2});
  C.watch(true);
  arr q0 = C.getJointState();

  //-- start a robot thread
  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.home(C);


  LeapMPC leap(C,1.);
  leap.komo.addObjective({1.}, FS_positionDiff, {"R_gripper", "target"}, OT_eq, {1e2});

  for(uint t=0;;t++){

    if(!(t%10)){
      switch(rnd(4)){
        case 0: target.setPosition(center + arr{+.3,.0,+.2}); break;
        case 1: target.setPosition(center + arr{-.3,.0,+.2}); break;
        case 2: target.setPosition(center + arr{+.3,.0,-.2}); break;
        case 3: target.setPosition(center + arr{-.3,.0,-.2}); break;
      }
    }

    double ctrlTime;
    arr q,qDot;
    {
      auto stateGet = bot.robot->state.get();
      ctrlTime = stateGet->time;
      q = stateGet->q;
      qDot = stateGet->qDot;
    }

    leap.reinit(C);
    leap.reinit(q, qDot); //zeros(q.N));
    leap.solve();
    rai::Graph R = leap.komo.getReport();


#if 1
    double T = bot.moveLeap(leap.xT);
    leap.komo.view(false, STRING("LEAP proposal T:"<<T <<"\n" <<R));
#else
    double constraints = R.get<double>("ineq") + R.get<double>("eq");
    double cost = R.get<double>("sos");
//    double T=10.; //leap.tau.last();
    double dist = length(q-leap.xT);
    double vel = scalarProduct(qDot, leap.xT-q)/dist;
    double alpha = .7;
    double T = (sqrt(6.*alpha*dist+vel*vel) - vel)/alpha;

    leap.komo.view(false, STRING("LEAP proposal T:"<<T <<"\n" <<R));

    if(T>.2 && cost<1. && constraints<1e-3){
#if 0
      double now=rai::realTime();
      {
        auto stateGet = robot.state.get();
        q = stateGet->q;
        qDot = stateGet->qDot;
      }
      arr _x = cat(q, leap.xT).reshape(2,-1);
      arr _t = {now, now+T};
      sp->overrideHardRealTime(_x, _t, qDot);
#else
      bot.moveOverride(~leap.xT, {T});
#endif
    }
#endif
    bot.step(C);
    if(bot.keypressed==13){ t=9; continue; }
    if(bot.keypressed=='q' || bot.keypressed==27) return;
    rai::wait(.1);
  }
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  testLeapCtrl();

  return 0;
}
