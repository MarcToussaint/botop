#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>
#include <Franka/FrankaGripper.h>

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  //-- setup a configuration
  rai::Configuration C;

  if(rai::checkParameter<rai::String>("confFile")){
    C.addFile(rai::getParameter<rai::String>("confFile"));
  }else{
    if(rai::getParameter<rai::String>("bot/useArm", "left")=="left"){
      C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
    }else{
      C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable-calibrated.g"));
    }
  }

  BotOp bot(C, !rai::checkParameter<bool>("sim"));

  if(rai::checkParameter<bool>("close")){
    bot.hold();
    for(auto g:bot.frankaGrippers) g->close();
    for(auto g:bot.frankaGrippers){ while(!g->isDone()) rai::wait(.1); }
  }

  if(rai::checkParameter<bool>("open")){
    bot.hold();
    for(auto g:bot.frankaGrippers) g->open();
    for(auto g:bot.frankaGrippers){ while(!g->isDone()) rai::wait(.1); }
  }

  if(rai::checkParameter<bool>("float")){
    bot.floating();
    bot.wait(C, true, false);
  }

  if(rai::checkParameter<bool>("hold")){
    bot.hold();
    bot.wait(C, true, false);
  }

  if(rai::checkParameter<bool>("up")){
    arr q=bot.qHome;
    q(1) -= .5;
    if(q.N>7) q(8) -=.5;
    bot.moveTo(q, 1.);
    bot.wait(C, true, true);
  }

  if(rai::checkParameter<bool>("loop")){
    arr q=bot.qHome;
    bot.moveTo(q, 1.);
    bot.wait(C, true, true);

    C.setJointState(bot.qHome);
    arr path = getLoopPath(C);
    bot.move(path, {5.});

    bot.wait(C, true, true);
  }

  if(rai::checkParameter<bool>("home")){
    arr q=bot.qHome;
    bot.moveTo(q, 1.);
    bot.wait(C, true, true);
  }

  cout <<"bye bye" <<endl;

  return 0;
}
