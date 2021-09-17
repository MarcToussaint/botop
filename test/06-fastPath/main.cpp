#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));

  arr path = getLoopPath(C);

  //-- start a robot thread
  BotOp bot(C, rai::checkParameter<bool>("real"));
  bot.home(C);
  //bot.setControllerWriteData(1);

  //prepare storing optitrack data ?
  ofstream fil("z.calib.dat");
  rai::Frame *optiFrameR=0, *optiFrameL = 0;
  if(bot.optitrack){
    rai::wait(1.);
    optiFrameR = C["ot_r_panda_gripper"];
    optiFrameL = C["ot_l_panda_gripper"];
  }

  //-- send path as spline
  for(double speed=1.;speed<=5.;speed+=.5){
    bot.move(path, ARR(10.)/speed);

    Metronome tic(.01);
    while(bot.step(C, -1.)){
      if(bot.optitrack){
        tic.waitForTic();
        arr q = bot.get_q();
        bot.optitrack->pull(C);
        fil <<rai::realTime() <<" q " <<q <<" poseL " <<optiFrameL->ensure_X() <<" poseR " <<optiFrameR->ensure_X() <<endl; // <<" poseTable " <<optiTable->get_X() <<endl;
      }
    }
    if(bot.keypressed=='q' || bot.keypressed==27) break;
  }
  fil.close();

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::getParameters()() <<'\n';

  return 0;
}
