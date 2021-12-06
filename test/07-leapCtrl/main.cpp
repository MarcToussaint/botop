#include <BotOp/bot.h>
#include <Control/LeapMPC.h>

//===========================================================================

void testLeapCtrl() {
  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));

  //add a target in position space
  arr center = C["l_gripper"]->getPosition();
  rai::Frame& target =
      C.addFrame("target")
      ->setShape(rai::ST_marker, {.1})
      .setPosition(center + arr{+.3,.0,+.2});
  C.watch(true);

  //-- start a robot thread
  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.home(C);

  //-- create a leap controller (essentially receeding horizon KOMO solver)
  LeapMPC leap(C,1.);
  leap.komo.addObjective({1.}, FS_positionDiff, {"l_gripper", "target"}, OT_eq, {1e2});

  //-- iterate (with wait(.1))
  for(uint t=0;;t++){
    rai::wait(.1);

    //-- switch target randomly every second
    if(!(t%10)){
      switch(rnd(4)){
        case 0: target.setPosition(center + arr{+.3,.0,+.2}); break;
        case 1: target.setPosition(center + arr{-.3,.0,+.2}); break;
        case 2: target.setPosition(center + arr{+.3,.0,-.2}); break;
        case 3: target.setPosition(center + arr{-.3,.0,-.2}); break;
      }
    }

    //get time,q,qDot - as batch from the same mutex lock
    double ctrlTime;
    arr q,qDot;
    {
      auto stateGet = bot.robotL->state.get();
      ctrlTime = stateGet->time;
      q = stateGet->q;
      qDot = stateGet->qDot;
    }

    //solve the leap problem
    leap.reinit(C); //adopt all frames in C as prefix (also positions of objects)
    leap.reinit(q, qDot);
    leap.solve();
    rai::Graph R = leap.komo.getReport();

    //send leap target
    double T = bot.moveLeap(leap.xT);
    leap.komo.view(false, STRING("LEAP proposal T:"<<T <<"\n" <<R));

    //update C
    bot.step(C);
    if(bot.keypressed==13){ t=9; continue; }
    if(bot.keypressed=='q' || bot.keypressed==27) return;
  }
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  testLeapCtrl();

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::getParameters()() <<'\n';

  return 0;
}
