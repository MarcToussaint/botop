#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  //-- setup a configuration
  rai::Configuration C;

  if(rai::checkParameter<rai::String>("confFile")){
    C.addFile(rai::getParameter<rai::String>("confFile"));
  }else{
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable-calibrated.g"));
  }

  BotOp bot(C, !rai::checkParameter<bool>("sim"));

  if(rai::checkParameter<bool>("open")){
    bot.hold(false, true);
    if(bot.gripperL) bot.gripperL->open();
    if(bot.gripperR) bot.gripperR->open();
    if(bot.gripperL) while(!bot.gripperL->isDone()) rai::wait(.1);
    if(bot.gripperR) while(!bot.gripperR->isDone()) rai::wait(.1);
  }

  if(rai::checkParameter<bool>("close")){
    bot.hold(false, true);
    if(bot.gripperL) bot.gripperL->close();
    if(bot.gripperR) bot.gripperR->close();
    if(bot.gripperL) while(!bot.gripperL->isDone()) rai::wait(.1);
    if(bot.gripperR) while(!bot.gripperR->isDone()) rai::wait(.1);
  }

  if(rai::checkParameter<bool>("float")){
    bot.hold(true, false);
    while(bot.step(C));
  }

  if(rai::checkParameter<bool>("damp")){
    bot.hold(true, true);
    while(bot.step(C));
  }

  if(rai::checkParameter<bool>("hold")){
    bot.hold(false, true);
    while(bot.step(C));
  }

  if(rai::checkParameter<bool>("home")){
    arr q=bot.qHome;
    bot.moveLeap(q, 1.);
    while(bot.step(C));
  }

  if(rai::checkParameter<bool>("up")){
    arr q=bot.qHome;
    q(1) -= .5;
    if(q.N>7) q(8) -=.5;
    bot.moveLeap(q, 1.);
    while(bot.step(C));
  }

  if(rai::checkParameter<bool>("loop")){
    arr q=bot.qHome;
    bot.moveLeap(q, 1.);
    while(bot.step(C));

    C.setJointState(bot.qHome);
    arr path = getLoopPath(C);
    bot.move(path, {5.});

    while(bot.step(C));
  }

  cout <<"bye bye" <<endl;

  return 0;
}
