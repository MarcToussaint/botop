#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  //-- setup a configuration
  rai::Configuration C;

  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));

  BotOp bot(C, !rai::checkParameter<bool>("sim"));

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

  if(rai::checkParameter<bool>("close")){
    bot.gripperL->close();
    rai::wait(.5);
  }

  if(rai::checkParameter<bool>("open")){
    bot.gripperL->open();
    rai::wait(.5);
  }

  cout <<"bye bye" <<endl;

  return 0;
}
