#include <KOMO/pathTools.h>

#include <BotOp/bot.h>
#include <BotOp/motionHelpers.h>

//===========================================================================

void rndPoses(){
  rai::Configuration C;
  bool writeData = rai::getParameter<bool>("bot/writeData", false);
  rai::String useArm = rai::getParameter<rai::String>("bot/useArm", "both");
  if(useArm=="both"){
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable.g"));
    C.addFrame("l_gripper_target") ->setShape(rai::ST_marker, {.2});
    C.addFrame("r_gripper_target") ->setShape(rai::ST_marker, {.2});
  }else{
    C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
    C.addFrame("l_gripper_target") ->setShape(rai::ST_marker, {.2});
  }

  double maxVel = rai::getParameter<double>("maxVel", 1.);
  double maxAcc = rai::getParameter<double>("maxAcc", 1.);

  arr qHome = C.getJointState();
  arr limits = C.getLimits();
  FrameL collisionPairs = C.getCollisionAllPairs();
  //for(uint i=0;i<collisionPairs.d0;i++) cout <<"PAIR: " <<collisionPairs(i,0)->name <<' ' <<collisionPairs(i,1)->name <<endl;

  BotOp bot(C, rai::getParameter<bool>("real", false));
  bot.home(C);
  arr bounds = ~C.getLimits();

  uint N=100;
  arr x_last = bot.get_q();
  for(uint i=0;i<N;i++){
    cout <<" ====== POSE " <<i <<" ====== " <<endl;
    arr x = bounds[0] + (bounds[1]-bounds[0]) % rand(qHome.N);

    C.setJointState(x);
    //C.view(false, STRING("conf " <<i));
    bool succ = PoseTool(C,0).checkLimitsAndCollisions(limits, {}, true);
    if(succ){
      cout <<" === pose made feasible === " <<endl;
      x = C.getJointState();
      //C.view(true, STRING("conf " <<i));

      //compute path
      C.setJointState(x_last);
      arr path = getStartGoalPath(C, x, bot.qHome);
      if(!path.N){
        cout <<" === path infeasible === " <<endl;
        continue;
      }
      x_last = x;
      cout <<" === path feasible  === " <<endl;

      //wait til finished and update gui
      bot.wait(C, false, true);
      if(bot.keypressed=='q') break;

      //write data
      if(writeData){
        bot.robotL->writeData=2;
        rai::wait(.05);
        bot.robotL->writeData=0;
      }

      cout <<" === -> executing === " <<endl;
      bot.moveAutoTimed(path, maxVel, maxAcc);
    }else{
      cout <<" === pose infeasible === " <<endl;
    }
  }
}

//===========================================================================

int main(int argc, char * argv[]){
  rai::initCmdLine(argc, argv);

//  rnd.clockSeed();
  rnd.seed(2);

  rndPoses();

  return 0;
}
