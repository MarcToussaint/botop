#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>
#include <OptiTrack/optitrack.h>

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

  //-- setup a configuration
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));

  arr path = getLoopPath(C);

  //-- start a robot thread
  BotOp bot(C, rai::getParameter<bool>("real", false));

  bot.setControllerWriteData(1);

  bot.home(C);

  //prepare storing optitrack data ?
  ofstream fil("z.calib.dat");
  rai::Frame *optiFrameR=0, *optiFrameL = 0;
  if(bot.optitrack){
    rai::wait(1.);
    optiFrameR = C["ot_r_panda_gripper"];
    optiFrameL = C["ot_l_panda_gripper"];
  }

  //-- send path as spline
  Metronome tic(.01);

  for(double speed=1.;speed<=3.;speed+=.5){
    bot.move(path, arr{10.}/speed);

    if(bot.optitrack){
      while(bot.sync(C, -1.)){
        tic.waitForTic();
        if(bot.optitrack){
          arr q = bot.get_q();
          bot.optitrack->pull(C);
          fil <<rai::realTime() <<" q " <<q <<" poseL " <<optiFrameL->ensure_X() <<" poseR " <<optiFrameR->ensure_X() <<endl; // <<" poseTable " <<optiTable->get_X() <<endl;
        }
      }
    }else{
      bot.wait(C, false, true);
    }
    if(bot.keypressed=='q' || bot.keypressed==27) break;
  }
  fil.close();

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::params() <<'\n';

  return 0;
}
